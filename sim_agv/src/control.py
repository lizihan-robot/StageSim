#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_forward():
    # 创建一个发布者，发布速度指令给小车
    pub = rospy.Publisher('/my_robot1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('control_node', anonymous=True)
    rate = rospy.Rate(10)  # 发布频率为10Hz

    while not rospy.is_shutdown():
        # 创建Twist消息
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5  # 设置线速度为0.5 m/s
        cmd_vel_msg.angular.z = 0.0  # 设置角速度为0.0 rad/s

        # 发布速度指令
        pub.publish(cmd_vel_msg)
        rate.sleep()

def rotate():
    # 创建一个发布者，发布速度指令给小车
    pub = rospy.Publisher('/my_robot1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('control_node', anonymous=True)
    rate = rospy.Rate(10)  # 发布频率为10Hz

    while not rospy.is_shutdown():
        # 创建Twist消息
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0  # 设置线速度为0.0 m/s
        cmd_vel_msg.angular.z = 0.5  # 设置角速度为0.5 rad/s

        # 发布速度指令
        pub.publish(cmd_vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # 运行小车运动控制函数
        move_forward()
        # rotate()
    except rospy.ROSInterruptException:
        pass
