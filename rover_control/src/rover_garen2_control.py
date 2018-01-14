#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin,cos, pi
import random
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String


class VelocityController(object):
    def __init__(self):
        rospy.init_node('rover_simulator_garen_controller')


        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0
        self.vy = 0
        self.vth = 0

        self.current_time =  rospy.Time.now()
        self.last_time =  rospy.Time.now()
        
        
        self.odom_pub = rospy.Publisher('/husky_velocity_controller/odom', Odometry, queue_size = 50)
        self.twist = Twist()
        self.controller()
    

    def callback(self,data):

        self.twist.linear.x = data.linear.x 
        self.twist.linear.y = data.linear.y 
        self.twist.angular.z = data.angular.z
  


    def controller(self):
        rate = rospy.Rate(10) #10 Hz

        while not rospy.is_shutdown():

            # for simulating
            self.vx = self.twist.linear.x
            self.vy = self.twist.linear.y
            self.vth = self.twist.angular.z

            self.current_time = rospy.Time.now()

            # We do not need this part, we are doing our own localization
            # compute odometry in a typical way given the velocities of the robot
            self.dt = (self.current_time - self.last_time).to_sec()
            self.th = self.vth*self.dt
            self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
            self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
           
            self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
   
            # next, we'll publish the odometry message over ROS
            self.odom = Odometry()
            self.odom.header.stamp = self.current_time
            self.odom.header.frame_id = "odom"

            # set the position
            self.odom.pose.pose = Pose(Point(self.delta_x , self.delta_y, 0.), Quaternion(*self.odom_quat))
            
            # Write a tranform formula for calculating linear.x linear.y and angular.z speed
            # set the velocity
            self.odom.child_frame_id = "base_link"
            self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
            # Subscriber(s)
           
            self.last_time = self.current_time
            rospy.Subscriber('/rover_imu/cmd_vel', Twist, self.callback)
            #rospy.Subscriber('/rover_serial_sensor',String, self.callback_sensor)
            # Publisher(s)
            self.odom_pub.publish(self.odom) 
            rate.sleep()
             

if __name__ == '__main__':
    VelocityController()
