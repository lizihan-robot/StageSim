#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from sim_agv.srv import AgvPose
from pid_control import PIDControl
import tf.transformations as tfm
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time
import threading
class AGV:
    def __init__(self, agv_num):
        self.agv_prefix = '/robot_'+ str(agv_num)
        self.pub_cmd = self.agv_prefix + '/cmd_vel'
        self.gp_service_name = self.agv_prefix + '/get_pose'
        self.res_cmd = '/reset_positions'
        rospy.wait_for_service(self.gp_service_name)
        self.vel_msg = Twist()
        self.rate = 50
        self.agv_pub = rospy.Publisher(self.pub_cmd, Twist, queue_size=10)
        self.gp_clinet = rospy.ServiceProxy(self.gp_service_name, AgvPose)
        self.reset_client = rospy.ServiceProxy(self.res_cmd, Empty)
        print("[AGV]: Create agv {}".format(agv_num))
        self.lock = threading.Lock()
        self.latest_pose = None

    def reset_positions(self):
        self.reset_client()

    def get_pose(self):
        request = self.gp_clinet()
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
    
    def move_l(self, tar_pose, speed=1.0):
        with self.lock:
            start = time.time()
            distance = 10
            angular_pid = PIDControl(Kp=0.1, Ki=0.000, Kd=0.002)
            speed_pid = PIDControl(Kp=0.05, Ki=0.0, Kd=0.0005)
            rospy.Rate(self.rate)
            agv_pose = self.get_pose()
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
            while distance>0.1:  
                agv_pose = self.get_pose()
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
                if distance<0.1:
                    speed = speed_pid.update(0, distance, self.rate)
                    #TODO 判断距离
                # print("distance: {}, cur_pose: {}, tar_pose: {}, speed: {}"\
                #                 .format(distance, agv_pose, tar_pose, speed))
                self.vel_msg.linear.x = speed
                self.agv_pub.publish(self.vel_msg)
                rospy.sleep(0.1)
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            # rospy.wait_for_message(self.pub_cmd, Twist) 
            self.agv_pub.publish(self.vel_msg)
            rospy.sleep(0.5)
            print("[AGV]: current pose {}".format(self.get_pose()))
        return time.time()-start

    def move_c(self, angle, speed_c=0.5, e=0.1):
        with self.lock:
            start = time.time()
            pid = PIDControl(Kp=0.1, Ki=0.0, Kd=0.0)
            rate = rospy.Rate(self.rate)
            # 计算角度差
            cur_angle = self.get_pose()[-1]
            tf_base_car = tfm.euler_matrix(0,0,math.radians(cur_angle),'sxyz')
            tf_car_new = tfm.euler_matrix(0,0,math.radians(angle),'sxyz')
            tf_base_new = np.dot(tf_base_car, tf_car_new)
            C = np.dot(np.linalg.inv(tf_base_car), tf_base_new)
            angle_dif = tfm.euler_from_matrix(C, 'sxyz')[-1]
            angle_dif = math.degrees(angle_dif)
            speed = speed_c
            while abs(angle_dif) > e:
                # 计算角度差
                cur_angle = self.get_pose()[-1]
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
                self.agv_pub.publish(self.vel_msg)
                # rate.sleep()
            self.vel_msg.angular.z = 0  
            self.agv_pub.publish(self.vel_msg)
            print("[AGV]: current pose {}".format(self.get_pose()))
        return time.time()-start

    def move_ofsc(self, tar_angle, speed_c=1, e=1):
        with self.lock:
            start = time.time()
            pid = PIDControl(Kp=0.1, Ki=0.0, Kd=0.0)
            rospy.Rate(self.rate)
            # 计算角度差
            cur_angle = self.get_pose()[-1]
            tf_base_car = tfm.euler_matrix(0,0,math.radians(cur_angle),'sxyz')
            tf_base_new = tfm.euler_matrix(0,0,math.radians(tar_angle),'sxyz')
            C = np.dot(np.linalg.inv(tf_base_car), tf_base_new)
            angle_dif = tfm.euler_from_matrix(C, 'sxyz')[-1]
            angle_dif = math.degrees(angle_dif)
            speed = speed_c
            while abs(angle_dif) > e:
                # 计算角度差
                cur_angle = self.get_pose()[-1]
                tf_base_car = tfm.euler_matrix(0,0,math.radians(cur_angle),'sxyz')
                tf_car_new = np.dot(np.linalg.inv(tf_base_car), tf_base_new)
                angle_dif = tfm.euler_from_matrix(tf_car_new, 'sxyz')[-1]
                angle_dif = math.degrees(angle_dif)
                # # use pid    
                if angle_dif < 1 :
                    speed = pid.update(0, angle_dif, 0.2)
                    speed = speed if angle_dif > 0 else -speed
                else:
                    speed = speed_c if angle_dif > 0 else -speed_c
                # print("angle_dif: {}  cur_angle: {} input tar_angle:{} speed: {}"\
                #       .format(angle_dif, cur_angle, tar_angle, speed))
                # speed = angle_dif / 180 * 3.14159
                # print("angle_dif: {}, speed: {}".format(angle_dif,speed))
                self.vel_msg.angular.z = speed
                # rospy.wait_for_message(self.pub_cmd, Twist) 
                self.agv_pub.publish(self.vel_msg)
                rospy.sleep(0.1)
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0  
            # rospy.wait_for_message(self.pub_cmd, Twist) 
            self.agv_pub.publish(self.vel_msg)
            rospy.sleep(0.5)
            print("[AGV]: current pose {}".format(self.get_pose()))
        return time.time()-start
    
class AGVDispath():
    def __init__(self, num):
        rospy.init_node('agv_dispath')
        # self.dispath_queue = Queue.Queue()
        self.dispath_list = []
        
        for i in range(num):
            agv = AGV(i)
            # self.dispath_queue.put(agv)
            self.dispath_list.append(agv)
        
    def get_dispath(self):
        """
        :rtype: list[AGV]
        """
        # return self.dispath_queue
        return self.dispath_list


if __name__=="__main__":
    rospy.init_node('agv_test')
    agv = AGV(0)
    agv.move_ofsc(90)
