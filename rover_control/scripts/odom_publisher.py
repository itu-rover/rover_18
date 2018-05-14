#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin,cos, pi,sqrt,pow
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String




class Localization(object):
	def __init__(self):
		rospy.init_node('odom_publisher')


		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.th = 0.0
		self.front_left=0.0
		self.front_right =0.0
		self.back_left=0.0
		self.back_right=0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vth = 0.0
		self.left_wheel=0.0
		self.right_wheel=0.0
		self.dist_btw_wheels=0.71
		self.current_time =  rospy.Time.now()
		self.last_time =  rospy.Time.now()
		self.odom_cur=Odometry()
		self.odom_pub = rospy.Publisher('/odometry/wheel', Odometry, queue_size = 10)
		rospy.Subscriber('/rover_serial_encoder', String, self.callback)
		self.controller()
	   
	 
 
	def callback(self,data):
		self.splitted=data.data.split(',')  

		if(self.splitted[0]=='S'):
			if(float(self.splitted[1])>=1000):
				self.front_left=(float(self.splitted[1])-1000)
			if(float(self.splitted[1])<1000):
				self.front_left=(-float(self.splitted[1]))

			if(float(self.splitted[2])>=1000):
				self.back_left=(float(self.splitted[2])-1000)
			if(float(self.splitted[2])<1000):
				self.back_left=(-float(self.splitted[2]))

			if(float(self.splitted[3])>=1000):
				self.front_right=(float(self.splitted[3])-1000)
			if(float(self.splitted[3])<1000):
				self.front_right=(-float(self.splitted[3]) )

			if(float(self.splitted[4])>=1000):
				self.back_right=(float(self.splitted[4])-1000)

			if(float(self.splitted[4])<1000):
				self.back_right=(-float(self.splitted[4]) )

			
			#print(str(self.front_left)+","+str(self.back_left)+","+str(self.front_right)+","+str(self.back_right))
	  
	def controller(self):
		self.rate = rospy.Rate(20) #10 Hz
		while not rospy.is_shutdown():
			self.current_time = rospy.Time.now()
			self.dt = (self.current_time - self.last_time).to_sec()

			self.left_wheel=((self.front_left+self.front_right)/2)*0.00601# front left front right
			self.right_wheel=((self.back_left+self.back_right)/2)*0.00601 #pi*0.115m 

			self.vx =  ((self.right_wheel+self.left_wheel)/2) 
			self.vth  = ((self.right_wheel-self.left_wheel)/self.dist_btw_wheels)


			self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
			self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
			self.delta_th = self.vth * self.dt
			self.x += self.delta_x
			self.y += self.delta_y
			self.th +=self.delta_th    

			print('distance: '+str(sqrt(pow(self.x,2)+pow(self.y,2)))+'yaw: '+str(self.th)+'vx:'+str(self.vx)+'vth:'+str(self.vth))
			self.q = tf.transformations.quaternion_from_euler(0, 0, self.th)
			# next, we'll publish the odometry message over ROS
			self.odom = Odometry()
			self.odom.header.stamp = self.current_time
			self.odom.header.frame_id = "odom"
			# set the position
			self.odom.pose.pose = Pose(Point(self.x , self.y, self.z), Quaternion(*self.q))
			self.odom.child_frame_id = "base_link"          
			self.last_time = self.current_time
			self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
			# Publisher(s) 
			#print(self.odom)
			self.odom_pub.publish(self.odom) 
			self.rate.sleep()     

if __name__ == '__main__':
	Localization()
