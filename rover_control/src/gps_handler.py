#!/usr/bin/env python
# -*- coding: utf-8 -*-


import math
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped,TwistWithCovarianceStamped
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import WayPoint, GeoPoint
 




class Sensor_Handler(object):


    def __init__(self):
        rospy.init_node("gps_node")
        self.pub = rospy.Publisher('gps/fix', NavSatFix, queue_size = 50 )
        self.pub1 = rospy.Publisher('rover_imu/cmd_vel_withcov',TwistWithCovarianceStamped, queue_size = 10)
        self.lat=0.0
        self.long=0.0 
        self.alt=0.0 
        self.odom_quat=[0.0 ,0.0,0.0,0.0]
        self.twcs=TwistWithCovarianceStamped()
        self.gps_broadcaster = tf.TransformBroadcaster()
        self.take_sensor_data()

    def callback_sensor(self,data):

        self.splitted_data=data.data.split(',') #serialdan gelen veri alındı
        if(self.splitted_data[1] !='' or self.splitted_data[2] !=''or self.splitted_data[3] !='' or  self.splitted_data[4] !=''):
            self.lat=(float(self.splitted_data[1]))
            self.long=(float(self.splitted_data[2]))
            self.alt=(float(self.splitted_data[3]))
            self.twcs.twist.twist.linear.x=(float(self.splitted_data[4]))
    def callback_odom(self,data):

        self.odom_quat=[data.pose.pose.orientation.x ,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        

    def take_sensor_data(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            self.addLat = 0.0
            self.addLon = 0.0
            self.avrLat = 0.0
            self.avrLon = 0.0
            self.current_time = rospy.Time.now()
            self.gps_fix = NavSatFix()

            self.gps_fix.header.frame_id = "gps"
            self.gps_fix.header.stamp = rospy.Time.now()
            self.gps_fix.status.status = 0 # GPS FIXED
            self.gps_fix.status.service = 1 # GPS SERVICE = GPS
            # Buralar bizden gelecek
            for x in range (0,100):
                self.addLat += self.lat
                self.addLon += self.long
            x=0
            self.avrLon = self.addLon/100
            self.avrLat = self.addLat/100
            print(str(self.avrLat) + "  " + str(self.avrLon) +   "  ")

            if (self.avrLon == 0 or self.avrLat == 0):
                print("0,0 ERROR")
            else:
                self.gps_fix.latitude =  self.avrLat
                self.gps_fix.longitude =  self.avrLon
                self.gps_fix.altitude = self.alt
                self.gps_fix.position_covariance = [0,0,0,0,0,0,0,0,0]
                self.gps_fix.position_covariance_type = 0
                self.pub.publish(self.gps_fix)
                self.pub1.publish(self.twcs)
             
            print(str(self.odom_quat))
            self.gps_broadcaster.sendTransform((0.0,0.0, 0.0),self.odom_quat,self.current_time,"gps","base_link")
            rospy.Subscriber('/rover_serial_sensor',String, self.callback_sensor)
            rospy.Subscriber('/odometry/filtered',Odometry, self.callback_odom)

            rate.sleep()
            
  

         
if __name__ == '__main__':
    Sensor_Handler()
