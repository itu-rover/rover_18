#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf
import random
from std_msgs.msg import String

imuMsg=Imu()

def callback_sensor(data):
   global imuMsg
   imuMsg=data

def main():
    global imuMsg
    
    rospy.init_node("imu_node")
    pub = rospy.Publisher('imu/data1', Imu, queue_size = 10)
    
   
    rate  = rospy.Rate(10)
    while not rospy.is_shutdown():
       

        print(imuMsg)
        pub.publish(imuMsg)
        
        rospy.Subscriber('/imu/data',Imu, callback_sensor)
        rate.sleep()




if __name__ == '__main__':
    main()