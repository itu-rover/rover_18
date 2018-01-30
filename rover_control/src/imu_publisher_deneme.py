#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf
import random
from std_msgs.msg import String

imuMsg=Imu()

def callback_imu(data):
    global imuMsg
    data_splitted=data.data.split(',')
    imuMsg.linear_acceleration.x =float(data_splitted[4])
    imuMsg.linear_acceleration.y = float(data_splitted[5])
    imuMsg.linear_acceleration.z = float(data_splitted[6])

    # Gyro
    imuMsg.angular_velocity.x = float(data_splitted[7])
    imuMsg.angular_velocity.y = float(data_splitted[8])
    imuMsg.angular_velocity.z =float(data_splitted[9])
 
    imuMsg.orientation.x = float(data_splitted[0])
    imuMsg.orientation.y = float(data_splitted[1])
    imuMsg.orientation.z =float(data_splitted[2])
    imuMsg.orientation.w = float(data_splitted[3])
        
def main():
    global imuMsg
    
    rospy.init_node("imu_node1")
    pub = rospy.Publisher('imu/data', Imu, queue_size = 10)
    
   
    rate  = rospy.Rate(10)
    while not rospy.is_shutdown():
       

        print(imuMsg)
        pub.publish(imuMsg)
        
        rospy.Subscriber('/rover_serial_imu',String, callback_imu)
        rate.sleep()




if __name__ == '__main__':
    main()