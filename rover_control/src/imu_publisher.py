#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf
import random
from std_msgs.msg import String

yaw=0
rover_accx=0

def callback_sensor(data):
    global yaw
    global rover_accx

    splitted_data=data.data.split(',') #serialdan gelen veri alındı
    if(splitted_data[0] !='' and splitted_data[1] !=''  ):
            #print(self.splitted_data[0]+","+self.splitted_data[1])
        rover_accx=(float(splitted_data[0]))
        yaw=(float(splitted_data[1])*3.14/180.0)

def main():

    

    rospy.init_node("imu_node")
    pub = rospy.Publisher('imu/data', Imu, queue_size = 10)
    global yaw
    global rover_accx
    yaw_old=yaw
    vth=0
    imuMsg = Imu()
    imuMsg.orientation_covariance = [0 , 0 , 0, 0, 0, 0, 0, 0, 0]
    imuMsg.angular_velocity_covariance = [0, 0 , 0, 0 , 0, 0, 0 , 0 , 0]
    imuMsg.linear_acceleration_covariance = [0 , 0 , 0, 0 , 0, 0, 0 , 0 , 0]
    current_time =  rospy.Time.now()
    last_time =  rospy.Time.now()
    print("Sleeping 1 second")
    rospy.sleep(1)
    # while(1)
    rate  = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Will be obtained from sensor
        current_time =  rospy.Time.now()
   
   
        dt = (current_time - last_time).to_sec()
       
        if(yaw>0):
            dyaw=yaw-yaw_old
            vth=dyaw/dt
        if(yaw<0):
            dyaw=yaw-yaw_old
            vth=dyaw/dt

        # Acceloremeter
        imuMsg.linear_acceleration.x = rover_accx
        imuMsg.linear_acceleration.y = 0
        imuMsg.linear_acceleration.z = 0

        # Gyro
        imuMsg.angular_velocity.x = 0
        imuMsg.angular_velocity.y = 0
        imuMsg.angular_velocity.z = vth

        q = tf.transformations.quaternion_from_euler(0,0,yaw)
        imuMsg.orientation.x = q[0] #magnetometer
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
        
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'base_link'
        rospy.loginfo(imuMsg)
        pub.publish(imuMsg)
        yaw_old=yaw
        last_time =  rospy.Time.now()
        rospy.Subscriber('/rover_serial_imu',String, callback_sensor)
        rate.sleep()




if __name__ == '__main__':
    main()