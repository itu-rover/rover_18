#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf
import random
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped
imuMsg = Imu()
twist=Twist()
twcs=TwistWithCovarianceStamped()


def callback_sensor(data):
    global imuMsg
    imuMsg=data


     




def main():

    

    rospy.init_node("imu_node")
    pub = rospy.Publisher('rover_imu/cmd_vel',Twist, queue_size = 10)
    pub1 = rospy.Publisher('rover_imu/cmd_vel_withcov',TwistWithCovarianceStamped, queue_size = 10)
    current_time =  rospy.Time.now()
    last_time =  rospy.Time.now()
    global imuMsg
    global twcs
    global twist
    vx=0
    vth=0
    print("Sleeping 1 second")
    rospy.sleep(1)
    # while(1)
    rate  = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()
        
        range=0.05
       

        if(imuMsg.linear_acceleration.x>0 and imuMsg.linear_acceleration.x<range):
            imuMsg.linear_acceleration.x=0
            vx=0

        if(imuMsg.linear_acceleration.x<0 and imuMsg.linear_acceleration.x>-range):
            imuMsg.linear_acceleration.x=0
            vx=0
        
        if(imuMsg.angular_velocity.z>0 and imuMsg.angular_velocity.z<range):
            imuMsg.angular_velocity.z=0
            vth=0
        if(imuMsg.angular_velocity.z<0 and imuMsg.angular_velocity.z>-range):
            imuMsg.angular_velocity.z=0
            vth=0

       
        vx += imuMsg.linear_acceleration.x*dt*50
        if(vx<0):
            vx=0

        vth = imuMsg.angular_velocity.z

        twcs.header.stamp =current_time
        twcs.header.frame_id = "odom"

        twcs.twist.twist.linear.x=vx
        twcs.twist.twist.angular.z=vth

        twist.linear.x=vx
        twist.angular.z=vth
 
   


        last_time = current_time
         
        pub.publish(twist)
        #pub1.publish(twcs)

        rospy.Subscriber('/imu/data',Imu, callback_sensor)
        rate.sleep()




if __name__ == '__main__':
    main()