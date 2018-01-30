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





class Sensor_Handler(object):


    def __init__(self):
        rospy.init_node("gps_node1")
        self.pub = rospy.Publisher('gps/goal', NavSatFix, queue_size = 50 )
        self.lat=0.0
        self.long=0.0 
        self.odom_quat=[0.0 ,0.0,0.0,0.0]
        self.flag=1
        self.gps_waypoint=NavSatFix()
        self.gps_broadcaster1 = tf.TransformBroadcaster()
        self.wayPointPublisher = rospy.Publisher('/desiredWayPoint', String, queue_size=10)
        self.longitude_old=0.0
        self.latitude_old=0.0
        self.flag=0.0
        self.take_sensor_data()

        

    def callback_odom(self,data):

        self.odom_quat=[data.pose.pose.orientation.x ,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]

    def callback_gps(self,data):

        self.gps_waypoint.latitude=data.latitude
        self.gps_waypoint.longitude=data.longitude
        
    def take_sensor_data(self):
        rate = rospy.Rate(10) # 10hz
	   
        while not rospy.is_shutdown():

  
            self.current_time = rospy.Time.now()
            self.gps_fix = NavSatFix()

            self.gps_fix.header.frame_id = "gps_goal"
            self.gps_fix.header.stamp = rospy.Time.now()
            self.gps_fix.status.status = 0 # GPS FIXED
            self.gps_fix.status.service = 1 # GPS SERVICE = GPS
        
            self.gps_fix.latitude =  self.gps_waypoint.latitude
            self.gps_fix.longitude = self.gps_waypoint.longitude
            self.gps_fix.altitude = 0
            self.gps_fix.position_covariance = [0,0,0,0,0,0,0,0,0]
            self.gps_fix.position_covariance_type = 0
            
            #self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
	        
           
            self.gps_broadcaster1.sendTransform((0.0,0.0, 0.0),self.odom_quat,self.current_time,"gps_goal","base_link")

           

            if(self.gps_fix.latitude != 0.0 or self.gps_fix.longitude != 0.0  ):
                if(self.flag<2): # eğer gelen data sayısı 2 olanaa kadar içine gir
               
                    self.wayPointPublisher.publish(str(self.gps_fix.latitude)+","+str(self.gps_fix.longitude))
                    self.pub.publish(self.gps_fix)
                    print(str(self.gps_fix.latitude)+","+str(self.gps_fix.longitude))


            
            if(self.gps_fix.latitude != 0.0 or self.gps_fix.longitude != 0.0 ): #gelen data 0 değil ve aynı data ise flagi arttır
                    if(self.gps_fix.longitude == self.longitude_old or self.gps_fix.latitude == self.latitude_old ):
                        self.flag = self.flag+1
                        #print(str(self.flag))
            if((self.gps_fix.latitude != 0.0 or self.gps_fix.longitude != 0.0 )and self.flag>2): #gelen data değişirse flagı 0 yapar
                if(self.gps_fix.longitude != self.longitude_old or self.gps_fix.latitude != self.latitude_old ):
                    self.flag = 0
                   

               
            
            self.longitude_old=self.gps_fix.longitude
            self.latitude_old=self.gps_fix.latitude
            

            rospy.Subscriber('/odometry/filtered',Odometry, self.callback_odom)
            rospy.Subscriber('/rover_gps/waypoint',NavSatFix, self.callback_gps)

            rate.sleep()
            
  

         
if __name__ == '__main__':
    Sensor_Handler()
