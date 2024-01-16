#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sim_agv.srv import AgvPose, AgvPoseResponse


class Turtlebot():
    def __init__(self, agv_num=0):
        self.agv_prefix = '/robot_'+ str(agv_num)
        self.sub_cmd = '/base_pose_ground_truth'
        self.transmit_pose = '/get_pose'
        self.pre_pose = []
        self.agv_pose = []
        # print("agv_pub: {}, self.agv_sub: {}".format(self.agv_prefix+self.pub_cmd, self.agv_prefix+self.sub_cmd))
        self.rate = 0.5
         
    def _odom_callback(self, msg):
        _odom = msg.pose.pose
        angle = euler_from_quaternion([
            _odom.orientation.x,
            _odom.orientation.y,
            _odom.orientation.z,
            _odom.orientation.w,
        ])
        self.agv_pose = [_odom.position.x,
                         _odom.position.y,
                         math.degrees(angle[-1])]

    def subscrib_agv_pose(self):
        rate = rospy.Rate(self.rate)
        self.agv_sub = rospy.Subscriber(self.agv_prefix+self.sub_cmd, Odometry, self._odom_callback)
        
        
    def transmit_pose_callback(self, req):
        res = AgvPoseResponse()
        if self.pre_pose == self.agv_pose:
            print("[Turtlebot]: Error agv {} get same pose {}"\
                  .format(self.agv_prefix, self.agv_pose))
        self.pre_pose = self.agv_pose
        res.x = self.agv_pose[0]
        res.y = self.agv_pose[1]
        res.angle = self.agv_pose[2]
        return res
    
      
    def transmit_server(self):
        s1 = rospy.Service(self.agv_prefix+self.transmit_pose, AgvPose, self.transmit_pose_callback)
        print("Ready to get {} pose from server".format(self.agv_prefix))
        assert self.agv_pose is not None, "self.agv_pose is None" 

    def run(self):
        self.subscrib_agv_pose()
        self.transmit_server()

if __name__ == "__main__":
    rospy.init_node('agv_control_node', anonymous=True)
    agv_num = 72
    for i in range(agv_num):
        agv = Turtlebot(i)
        agv.run()
    rospy.spin()