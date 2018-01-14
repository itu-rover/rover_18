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
        rospy.init_node("gps_node")
        self.pub = rospy.Publisher('gps/fix', NavSatFix, queue_size = 50 )
        self.lat=0.0
        self.long=0.0 
        #self.gps_broadcaster = tf.TransformBroadcaster()
        self.take_sensor_data()

    def callback_sensor(self,data):

        self.splitted_data=data.data.split(',') #serialdan gelen veri alındı
        if(self.splitted_data[1] !='' and self.splitted_data[2] !=''  ):
            self.lat=(float(self.splitted_data[1]))
            self.long=(float(self.splitted_data[2]))

    def take_sensor_data(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            self.current_time = rospy.Time.now()
            self.gps_fix = NavSatFix()

            self.gps_fix.header.frame_id = "base_link"
            self.gps_fix.header.stamp = rospy.Time.now()
            self.gps_fix.status.status = 0 # GPS FIXED
            self.gps_fix.status.service = 1 # GPS SERVICE = GPS
            # Buralar bizden gelecek
            self.gps_fix.latitude =  self.lat
            self.gps_fix.longitude =  self.long
            self.gps_fix.altitude = 0
            self.gps_fix.position_covariance = [0,0,0,0,0,0,0,0,0]
            self.gps_fix.position_covariance_type = 0
            self.pub.publish(self.gps_fix)
            #self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            #self.gps_broadcaster.sendTransform((0.0,0.0, 0.0),self.odom_quat,self.current_time,"base_link","odom")
            rospy.Subscriber('/rover_serial_sensor',String, self.callback_sensor)

            rate.sleep()
            
  

         
if __name__ == '__main__':
    Sensor_Handler()
