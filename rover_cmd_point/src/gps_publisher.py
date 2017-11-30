#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import tf
import random



def main():

    val  = random.random()
    # val = 0

    rospy.init_node("gps_node")
    pub = rospy.Publisher('odometry/gps', Imu, queue_size = 10)

    imuMsg = Imu()
    imuMsg.orientation_covariance = [0 , 0 , 0, 0, 0, 0, 0, 0, 0]
    imuMsg.angular_velocity_covariance = [0, 0 , 0, 0 , 0, 0, 0 , 0 , 0]
    imuMsg.linear_acceleration_covariance = [0 , 0 , 0, 0 , 0, 0, 0 , 0 , 0]

    print("Sleeping 1 second")
    rospy.sleep(1)
    # while(1)
    rate  = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Will be obtained from sensor
        roll = 0
        pitch = 0
        yaw = val
        # Acceloremeter
        imuMsg.linear_acceleration.x = 0
        imuMsg.linear_acceleration.y = 0
        imuMsg.linear_acceleration.z = 0

        # Gyro
        imuMsg.angular_velocity.x = 0
        imuMsg.angular_velocity.y = 0
        imuMsg.angular_velocity.z = 0

        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        imuMsg.orientation.x = q[0] #magnetometer
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
        
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'base_link'
        rospy.loginfo(imuMsg)
        pub.publish(imuMsg)
        rate.sleep()



if __name__ == '__main__':
    main()
