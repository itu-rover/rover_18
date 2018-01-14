#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist



class Manuel(object):
    def __init__(self):
        rospy.init_node('rover_manuel')
        self.pub=rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size=50)
        self.twist=Twist()
        self.loop()

    def loop(self):
        rospy.loginfo( "w,a,s,d , space for stop \n")
        linear_speed=0.5
        angular_speed=0.5
        while not rospy.is_shutdown():
            key_string = raw_input()
            if key_string=='w' :
                self.twist.linear.x=linear_speed
            if key_string=='s':
                self.twist.linear.x=-linear_speed

            if key_string=='a':
                self.twist.angular.z=angular_speed
            if key_string=='d':
                self.twist.angular.z=-angular_speed

            if key_string==' ':
                self.twist.linear.x=0
                self.twist.angular.z=0

            self.pub.publish(self.twist)


if __name__ == '__main__':
    Manuel()
    