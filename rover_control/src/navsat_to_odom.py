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
        rospy.init_node("gps_to_navsat")

        self.pub = rospy.Publisher('odometry/filtered', Odometry, queue_size = 50 )
        self.x=0.0
        self.y=0.0  
        self.th=0.0 
        self.flag=0 
        self.take_sensor_data()

    def callback_sensor(self,data):
        if(self.flag==0):
            self.flag == self.flag+1

        self.x=data.pose.pose.position.x
        self.y=data.pose.pose.position.y
        self.th=data.pose.pose.orientation.z

    def take_sensor_data(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            self.current_time = rospy.Time.now()
            self.odom = Odometry()
            self.odom.header.stamp = self.current_time
            self.odom.header.frame_id = "odom"
            self.odom.child_frame_id = "base_link"
            self.odom.pose.pose.position.x=0
            self.odom.pose.pose.position.y=0
            self.odom.pose.pose.orientation.z=0
            self.pub.publish(self.odom)

            #rospy.Subscriber('/odometry/gps',Odometry, self.callback_sensor)

            rate.sleep()
            
  

         
if __name__ == '__main__':
    Sensor_Handler()
