#! /usr/bin/env python
#coding=utf-8

import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    print('---------init vel_node-------')
    rospy.init_node('vel_node')
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()
    vel_msg.angular.z = 0.5
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        vel_pub.publish(vel_msg)
        rate.sleep()