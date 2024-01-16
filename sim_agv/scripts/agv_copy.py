#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from sim_agv.srv import AgvPose, AgvPoseResponse,AgvPoseRequest, \
                        MoveC, MoveCResponse,MoveCRequest, \
                        MoveL, MoveLResponse, MoveLRequest
from pid_control import PIDControl
import tf.transformations as tfm
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from std_srvs.srv import Empty
import time
import Queue
import threading
class AGVDispath:
    def __init__(self, agv_num):
        rospy.init_node('agv_dispath')
        self.pub_cmd = '/cmd_vel'
        self.res_cmd = '/reset_positions'
        self.lock = threading.Lock()
        self.rate = 1
        self.agv_list = []
        self.vel_msg = Twist()
        for i in range(agv_num):
            agv_prefix = '/robot_'+ str(i)
            gp_service_name = agv_prefix + '/get_pose'
            rospy.wait_for_service(gp_service_name)
            agv_pub = rospy.Publisher(agv_prefix+self.pub_cmd, Twist, queue_size=10)
            gp_clinet = rospy.ServiceProxy(gp_service_name, AgvPose)
            reset_client = rospy.ServiceProxy(self.res_cmd, Empty)
            print("[AGV]: Create agv {}".format(i))
            self.agv_list.append((agv_pub,gp_clinet,reset_client))

    def reset_positions(self, agv_num):
        self.agv_list[agv_num][2]()

    def get_pose(self,agv_num):
        request = self.agv_list[agv_num][1]()
        x = request.x
        y = request.y
        angle = request.angle
        return [x, y, angle]
    
    def pose_matrix(self, cur_pose, angle):
        pose = [cur_pose[0],cur_pose[1],0,0,0,angle]
        cur_matrix = np.eye(4)
        rot_matrix = tfm.euler_matrix(0,0,pose[-1],'sxyz')
        cur_matrix[:3,:3] = rot_matrix[:3,:3]
        cur_matrix[:2,3] = pose[:2]
        return cur_matrix

    def matrix_pose(self, matrix):
        """
        return pose:[x, y, angle]
        """
        pose = list(matrix[:2,3])
        angle = tfm.euler_from_matrix(matrix[:3,:3])
        pose.append(math.degrees(angle[-1]))
        return pose
    
    def move_l(self, agv_num, tar_pose, speed=1):
        with self.lock:
            start = time.time()
            distance = 10
            angular_pid = PIDControl(Kp=0.1, Ki=0.000, Kd=0.002)
            # speed_pid = PIDControl(Kp=0.05, Ki=0.0, Kd=0.0005)
            rate = rospy.Rate(self.rate)
            agv_pose = self.get_pose(agv_num)
            car_radians = np.radians(agv_pose[-1])
            rotation_matrix = np.array([[np.cos(car_radians), -np.sin(car_radians), 0],
                                        [np.sin(car_radians), np.cos(car_radians), 0],
                                        [0, 0, 1]])
            vector_ = np.array(
                [tar_pose[0] - agv_pose[0], tar_pose[1] - agv_pose[1], 0])
            # 将向量差转换到B坐标系中
            transformed_vector = np.dot(rotation_matrix.T, vector_)
            dif_angle = np.degrees(np.arctan2(
                        transformed_vector[1], transformed_vector[0]))
            # tf_base_car = self.pose_matrix(agv_pose[0:2], agv_pose[-1])
            # tf_base_new = self.pose_matrix(tar_pose, agv_pose[-1])
            # tf_car_new = np.dot(np.linalg.inv(tf_base_car), tf_base_new)
            # distance = math.sqrt(tf_car_new[0,3] ** 2 + tf_car_new[1,3] ** 2)
            while distance>0.01:  
                agv_pose = self.get_pose(agv_num)
                car_radians = np.radians(agv_pose[-1])
                rotation_matrix = np.array([[np.cos(car_radians), -np.sin(car_radians), 0],
                                            [np.sin(car_radians), np.cos(car_radians), 0],
                                            [0, 0, 1]])
                vector_ = np.array(
                    [tar_pose[0] - agv_pose[0], tar_pose[1] - agv_pose[1], 0])
                # 将向量差转换到B坐标系中
                transformed_vector = np.dot(rotation_matrix.T, vector_)
                dif_angle = np.degrees(np.arctan2(
                            transformed_vector[1], transformed_vector[0]))
                distance = np.linalg.norm(vector_)
                angular = angular_pid.update(0, dif_angle, self.rate)
                self.vel_msg.angular.z = -angular
                # speed =abs(speed) if pose_car_new[1] > 0 else -abs(speed)
                # print("distance: {}, cur_pose: {}, tar_pose: {}, speed: {}"\
                #                 .format(distance, agv_pose, tar_pose, speed))
                # if distance<0.1:
                    # speed = speed_pid.update(0, distance, self.rate)
                    #TODO 判断距离
                self.vel_msg.linear.x = speed
                while not rospy.is_shutdown():
                    self.agv_list[agv_num][0].publish(self.vel_msg)
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            while not rospy.is_shutdown():
                self.agv_list[agv_num][0].publish(self.vel_msg)
            rate.sleep()
        return time.time()-start

    def move_c(self,agv_num, angle, speed_c=0.5, e=0.1):
        with self.lock:
            start = time.time()
            pid = PIDControl(Kp=0.05, Ki=0.0, Kd=0.0005)
            rate = rospy.Rate(self.rate)
            # 计算角度差
            cur_angle = self.get_pose(agv_num)[-1]
            tf_base_car = tfm.euler_matrix(0,0,math.radians(cur_angle),'sxyz')
            tf_car_new = tfm.euler_matrix(0,0,math.radians(angle),'sxyz')
            tf_base_new = np.dot(tf_base_car, tf_car_new)
            C = np.dot(np.linalg.inv(tf_base_car), tf_base_new)
            angle_dif = tfm.euler_from_matrix(C, 'sxyz')[-1]
            angle_dif = math.degrees(angle_dif)
            speed = speed_c
            while abs(angle_dif) > e:
                # 计算角度差
                cur_angle = self.get_pose(agv_num)[-1]
                tf_base_car = tfm.euler_matrix(0,0,math.radians(cur_angle),'sxyz')
                tf_car_new = np.dot(np.linalg.inv(tf_base_car), tf_base_new)
                angle_dif = tfm.euler_from_matrix(tf_car_new, 'sxyz')[-1]
                angle_dif = math.degrees(angle_dif)

                if angle_dif < 1 :
                    speed = pid.update(0, angle_dif, self.rate)
                    speed = speed if angle_dif > 0 else -speed
                else:
                    speed = speed_c if angle_dif > 0 else -speed_c
                # print("angle_dif: {}  cur_angle: {} input angle:{} speed: {}".format(angle_dif, cur_angle, angle, speed))
                self.vel_msg.angular.z = speed
                while not rospy.is_shutdown():
                    self.agv_list[agv_num][0].publish(self.vel_msg)
            self.vel_msg.angular.z = 0  
            while not rospy.is_shutdown():
                self.agv_list[agv_num][0].publish(self.vel_msg)
            print("[AGV]: current pose {}".format(self.get_pose(agv_num)))
            rate.sleep()
        return time.time()-start

if __name__=="__main__":

    
    # import pdb;pdb.set_trace()
    dispath= AGVDispath(12)
    agv = dispath.get_dispath()
    agv[1].reset_positions()
    print(agv[1].get_pose())
    tar_pose = [0.0, 5.8, 90.0]
    agv[1].move_l(tar_pose, 1)
    # agv.move_c(-90,0.5)



