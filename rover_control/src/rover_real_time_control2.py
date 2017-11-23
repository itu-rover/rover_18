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
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.twist = Twist()
        self.controller()
        
    
    def callback(self,data):

        self.twist.linear.x = data.linear.x 
        self.twist.linear.y = data.linear.y 
        self.twist.angular.z = data.angular.z

    def controller(self):
        rate = rospy.Rate(10) #10 Hz

        while not rospy.is_shutdown():

            
            self.vx = self.twist.linear.x #it will  come from imu linear x 
            self.vy = self.twist.linear.y #it will come from imu linear y 
            self.vth = self.twist.angular.z #it will come from imu yaw
 
            self.current_time = rospy.Time.now()

            # We do not need this part, we are doing our own localization
            # compute odometry in a typical way given the velocities of the robot
            self.dt = (self.current_time - self.last_time).to_sec()
            self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
            self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
            self.delta_th = self.vth * self.dt
            
            self.x += self.delta_x
            self.y += self.delta_y
            self.th += self.delta_th

            self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
             
            self.odom_broadcaster.sendTransform((self.x, self.y, 0.),self.odom_quat,self.current_time,"base_link","odom")

            # next, we'll publish the odometry message over ROS
            self.odom = Odometry()
            self.odom.header.stamp = self.current_time
            self.odom.header.frame_id = "odom"

            # set the position
            self.odom.pose.pose = Pose(Point(self.x , self.y, 0.), Quaternion(*self.odom_quat))
            
            # Write a tranform formula for calculating linear.x linear.y and angular.z speed
            # set the velocity
            self.odom.child_frame_id = "base_link"
            self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
            # Subscriber(s)
           
            self.last_time = self.current_time
            rospy.Subscriber('/husky_velocity_controller/cmd_vel', Twist, self.callback)
            #rospy.Subscriber('/rover_serial_sensor',String, self.callback_sensor)
            # Publisher(s)
            self.odom_pub.publish(self.odom) 
            rate.sleep()
             

if __name__ == '__main__':
    Localization()
