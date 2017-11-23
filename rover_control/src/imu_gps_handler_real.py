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


latCur = 0
lonCur = 0

class Sensor_Handler(object):


    def bearing(self,latCur, lonCur, latWP, lonWP): #Bearing to waypoint (degrees)
        latWP, lonWP, latCur, lonCur = map(radians, [latWP, lonWP, latCur, lonCur]) #Convert into Radians to perform math
        dLon = lonWP - lonCur

        return atan2(sin(dLon) * cos(latWP), cos(latCur) * sin(latWP) - (sin(latCur) * cos(latWP) * cos(dLon)))

    def haversineDistance(self,latCur, lonCur, latWP, lonWP): 

        latWP, lonWP, latCur, lonCur = map(radians, [latWP, lonWP, latCur, lonCur]) #Convert into Radians to perform math
        a = pow(sin((latWP - latCur)/2),2) + cos(latCur) * cos(latWP) * pow(sin((lonWP - lonCur)/2),2)
        return self.earthRadius * 2.0 * asin(sqrt(a))  #Return calculated distance to waypoint in Metres
    def callback_odom(self,data):
        self.Xstr=str(data.pose.pose.position.x)
        self.Ystr=str(data.pose.pose.position.y)
        if(self.Xstr != '' and  self.flag==1):
            self.send_msgs.currPosY=self.Ystr
            self.send_msgs.currPosX=self.Xstr 
        self.flag=0
                
    def callback_sensor(self,data):
        self.data_splitted=data.data.split(',')
        global latCur
        latCur=float(self.data_splitted[1])
        global lonCur
        lonCur=float(self.data_splitted[2])
        


    def __init__(self):
        rospy.init_node('imu_gps_handler')
        self.pub= rospy.Publisher('/distancewithangle', Distancewithangle, queue_size = 50)
        self.flag=1
        self.WPupdatestate=0
        self.take_sensor_data()

    def take_sensor_data(self):
        rate = rospy.Rate(10) # 10hz
         
        while not rospy.is_shutdown():
            self.earthRadius = 6371000.0 #Metres
            
            self.current_time = rospy.Time.now()
            
            global latCur
            global lonCur

            if( self.WPupdatestate==0):
                self.latWP=  41.106113 #float(raw_input("Enter Goal Lat:") )          #self.waypoint.position.latitude
                self.lonWP= 29.024930 #float(raw_input("Enter Goal Lon:")  )         #self.waypoint.position.longitude
                self.WPupdatestate=1
                   

            

            rospy.Subscriber('/husky_velocity_controller/odom', Odometry ,self.callback_odom)
            rospy.Subscriber('/rover_serial_sensor', String ,self.callback_sensor)
             
            self.haversineDistance(latCur,lonCur,self.latWP,self.lonWP)

            self.bearing(latCur,lonCur,self.latWP,self.lonWP)

            self.send_msgs=Distancewithangle()
            self.send_msgs.distance=str(self.haversineDistance(latCur, lonCur, self.latWP, self.lonWP))
            self.send_msgs.angle=str(self.bearing(latCur, lonCur, self.latWP, self.lonWP))
            self.send_msgs.currPosX='0'
            self.send_msgs.currPosY='0'

            if(self.send_msgs.currPosX !=''):
                self.pub.publish(self.send_msgs)
                rospy.loginfo(self.send_msgs.currPosX)
                self.WPupdatestate=0

               


   

  

         
if __name__ == '__main__':
    Sensor_Handler()
