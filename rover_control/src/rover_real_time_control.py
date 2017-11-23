#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin,cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import WayPoint, GeoPoint



class Localization(object):
    def __init__(self):
        rospy.init_node('rover_real_time_control')


        self.x = 0.0
        self.y = 0.0
        self.th = 0.0


        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0


        self.current_time =  rospy.Time.now()
        self.last_time =  rospy.Time.now()
        
        self.odom_pub = rospy.Publisher('/husky_velocity_controller/odom', Odometry, queue_size = 50)
      
        self.twist = Twist()
        self.controller()
        
      

    def callback_odom(self,data):
       self.odom.pose.pose = data.pose.pose
       self.x = data.pose.pose.position.x
       self.y = data.pose.pose.position.y
       self.th = data.pose.pose.orientation.z
    def callback_cmd(self,data):
        
        self.twist.linear.x = data.linear.x 
        self.twist.linear.y = data.linear.y 
        self.twist.angular.z = data.angular.z

    def controller(self):
        rate = rospy.Rate(10) #10 Hz

        while not rospy.is_shutdown():
            
            self.vx= self.twist.linear.x 
            self.vy= self.twist.linear.y 
            self.th= self.twist.angular.z

            self.current_time = rospy.Time.now()
            self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)


            self.odom = Odometry()
            self.odom.header.stamp = self.current_time
            self.odom.header.frame_id = "odom"
            self.odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))
            
            self.odom.child_frame_id = "base_link"
            self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
          
             
            rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odom)
            rospy.Subscriber('/husky_velocity_controller/cmd_vel', Twist, self.callback_cmd)
            
         
            # Publisher(s)
            self.odom_pub.publish(self.odom) 

            rate.sleep()

             

if __name__ == '__main__':
    Localization()
