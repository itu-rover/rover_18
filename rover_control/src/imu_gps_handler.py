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
        rospy.init_node("imu_node")
        self.pub = rospy.Publisher('imu/data', Imu, queue_size = 10)

     
        self.yaw=0.0
        self.rover_accx=0.0
        self.take_sensor_data()

    def callback_sensor(self,data):

        self.splitted_data=data.data.split(',') #serialdan gelen veri alındı
        if(self.splitted_data[0] !='' and self.splitted_data[1] !=''  ):
            #print(self.splitted_data[0]+","+self.splitted_data[1])
            self.rover_accx=(float(self.splitted_data[0]))
            self.yaw=(float(self.splitted_data[1])*3.14/180.0)

    def take_sensor_data(self):
        rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Acceloremeter
        self.imuMsg = Imu()
        self.imuMsg.orientation_covariance = [0 , 0 , 0, 0, 0, 0, 0, 0, 0]
        self.imuMsg.angular_velocity_covariance = [0, 0 , 0, 0 , 0, 0, 0 , 0 , 0]
        self.imuMsg.linear_acceleration_covariance = [0 , 0 , 0, 0 , 0, 0, 0 , 0 , 0]
        self.imuMsg.linear_acceleration.x = 1
        self.imuMsg.linear_acceleration.y = 0
        self.imuMsg.linear_acceleration.z = 0

        # Gyro
        self.imuMsg.angular_velocity.x = 0
        self.imuMsg.angular_velocity.y = 0
        self.imuMsg.angular_velocity.z = 0

        q = tf.transformations.quaternion_from_euler(0,0,self.yaw)
        self.imuMsg.orientation.x = q[0] #magnetometer
        self.imuMsg.orientation.y = q[1]
        self.imuMsg.orientation.z = q[2]
        self.imuMsg.orientation.w = q[3]
        
        self.imuMsg.header.stamp= rospy.Time.now()
        self.imuMsg.header.frame_id = 'base_link'
        rospy.loginfo(self.imuMsg)
        self.pub.publish(self.imuMsg)

        rospy.Subscriber('/rover_serial_imu',String, self.callback_sensor)

        rate.sleep()

   
       
  

  

         
if __name__ == '__main__':
    Sensor_Handler()
