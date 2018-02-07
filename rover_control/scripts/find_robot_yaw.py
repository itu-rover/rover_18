#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf
import random
from std_msgs.msg import String
 
from nav_msgs.msg import Odometry

yaw=0
Msg=Odometry()

def callback_sensor(data):
    global Msg
    global yaw 
    Msg=data
    quaternion = (
    Msg.pose.pose.orientation.x,
    Msg.pose.pose.orientation.y,
    Msg.pose.pose.orientation.z,
    Msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    print(str(yaw*180/3.14))
def main():
    global yaw
    pub = rospy.Publisher('/imu/yaw', String, queue_size = 5 )

    rospy.init_node("imu_node1")
   
    rate  = rospy.Rate(10)
    while not rospy.is_shutdown():
        

        pub.publish(str(yaw*180/3.14))
        rospy.Subscriber('/odometry/filtered',Odometry, callback_sensor)
        rate.sleep()




if __name__ == '__main__':
    main()