#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin,cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import WayPoint, GeoPoint



class Localization(object):
    def __init__(self):
        rospy.init_node('rover_real_time_control3')


        self.x = 0.0
        self.y = 0.0
        self.th = 0.0


        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.rover_accx=0.0
        self.yaw=0.0

  
        self.current_time =  rospy.Time.now()
        self.last_time =  rospy.Time.now()
        
        self.odom_pub = rospy.Publisher('/husky_velocity_controller/odom', Odometry, queue_size = 50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.twist = Twist()
        self.controller()

        self.ferhat_pub=rospy.Publisher('ferhat_topic',String,queue_size=50)
        
    def callback_sensor(self,data):
        self.splitted_data=data.data.split(',') #serialdan gelen veri alındı
        if(self.splitted_data[0] !='' and self.splitted_data[1] !=''  ):
            #print(self.splitted_data[0]+","+self.splitted_data[1])
            self.rover_accx=(float(self.splitted_data[0]))
            self.yaw=(float(self.splitted_data[1])*3.14/180.0)

        
        

    def callback(self,data):

        self.twist.linear.x = data.linear.x 
        self.twist.linear.y = data.linear.y 
        self.twist.angular.z = data.angular.z

    def controller(self):
        self.rate = rospy.Rate(5) #10 Hz

        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()
                    
            
            if(self.rover_accx>0 and self.rover_accx<0.10): #belirli bir ivmeden sonrası 0 kabul edildi
                self.rover_accx=0
                self.vx = 0
            if(self.rover_accx<0 and self.rover_accx>-0.10): #belirli bir ivmeden sonrası 0 kabul edildi
                self.rover_accx=0
                self.vx = 0

            if(self.twist.linear.x != 0):
                self.vx=(self.rover_accx)*self.dt
            else:
                self.vx=0
                self.rover_accx = 0

           
            self.vy = self.twist.linear.y 

            self.th = self.yaw

            self.vth=self.twist.angular.z 
       

            self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
            self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
            #self.delta_th = self.vth * self.dt
            
            self.x += self.delta_x
            self.y += self.delta_y
   
            #print("vx:"+str(self.vx)+","+"x:"+str(self.x)+","+"dx:"+str(self.delta_x)+"dt:"+str(self.dt))
               
            self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
             
            self.odom_broadcaster.sendTransform((self.x, self.y, 0.0),self.odom_quat,self.current_time,"base_link","odom")

            # next, we'll publish the odometry message over ROS
            self.odom = Odometry()
            self.odom.header.stamp = self.current_time
            self.odom.header.frame_id = "odom"

            # set the position
            self.odom.pose.pose = Pose(Point(self.x , self.y, 0.0), Quaternion(*self.odom_quat))
            
            # Write a tranform formula for calculating linear.x linear.y and angular.z speed
            # set the velocity
            self.odom.child_frame_id = "base_link"
            self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
            # Subscriber(s)
           
            self.last_time = self.current_time
            rospy.Subscriber('/husky_velocity_controller/cmd_vel', Twist, self.callback)
            rospy.Subscriber('/rover_serial_imu',String, self.callback_sensor)
            # Publisher(s)
            print(self.odom)
            self.odom_pub.publish(self.odom)
            self.ferhat_pub.publish("vx = " + str(self.vx) + "  acc_x = " + str(self.acc_x) + "  twist.linear.x =  " + str(self.twist.linear.x))

            #print(str(self.yaw_min)+ "k:"+str(self.yaw_now)+"e"+str(self.yaw_max))
            self.rate.sleep()

            
             

if __name__ == '__main__':
    Localization()
