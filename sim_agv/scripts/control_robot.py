#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
import time
from geometry_msgs.msg import Twist

def move():
    rospy.init_node('robot_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # 设置线速度和角速度

    vel_msg.linear.x = 0.1
    # vel_msg.linear.y = 0.1
    # vel_msg.angular.z = 0.2
    start = time.time()
    while time.time() - start < 4:
        velocity_publisher.publish(vel_msg)
    vel_msg.linear.z = 0.0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass