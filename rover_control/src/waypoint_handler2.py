#!/usr/bin/env python
# -*- coding: utf-8 -*-


import math
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import WayPoint, GeoPoint
from rover_control.msg import Distancewithangle




class Sensor_Handler(object):


    def __init__(self):
        rospy.init_node("gps_node1")
        self.pub = rospy.Publisher('gps/goal', NavSatFix, queue_size = 50 )
        self.lat=0.0
        self.long=0.0 
        self.odom_quat=[0.0 ,0.0,0.0,0.0]
        self.gps_broadcaster = tf.TransformBroadcaster()
        self.take_sensor_data()

    def callback_odom(self,data):

        self.odom_quat=[data.pose.pose.orientation.x ,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        

    def take_sensor_data(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

  
            self.current_time = rospy.Time.now()
            self.gps_fix = NavSatFix()

            self.gps_fix.header.frame_id = "gps_goal"
            self.gps_fix.header.stamp = rospy.Time.now()
            self.gps_fix.status.status = 0 # GPS FIXED
            self.gps_fix.status.service = 1 # GPS SERVICE = GPS
        
            self.gps_fix.latitude =  41.106154
            self.gps_fix.longitude =  29.025409
            self.gps_fix.altitude = 0
            self.gps_fix.position_covariance = [0,0,0,0,0,0,0,0,0]
            self.gps_fix.position_covariance_type = 0
            self.pub.publish(self.gps_fix)
            #self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            
            self.gps_broadcaster.sendTransform((0.0,0.0, 0.0),self.odom_quat,self.current_time,"gps_goal","base_link")
            
            rospy.Subscriber('/odometry/filtered',Odometry, self.callback_odom)

            rate.sleep()
            
  

         
if __name__ == '__main__':
    Sensor_Handler()
