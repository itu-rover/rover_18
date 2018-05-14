#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#1-3 are the left side of the rover, 2-4 are the right side of the rover.

twist =Twist();
pub=rospy.Publisher("/rover_serial_topic", String, queue_size=50)
def callbackcmd(data):  
	twist.linear.x = data.linear.x 
	twist.angular.z = data.angular.z

def callbacknav(data):
	twist.linear.x = data.linear.x 
	twist.angular.z = -data.angular.z
	
def main():

	rospy.init_node('rover_cmd_sub_serial')
	rospy.Subscriber("/rover_joy/cmd_vel", Twist, callbackcmd)
	rospy.Subscriber("/rover_navigation/cmd_vel", Twist, callbacknav)
	b=0.71
	leftWheel =0
	rightWheel=0
	cmdRadius =0
	
	while not rospy.is_shutdown():
		if(twist.angular.z != 0):

			cmdRadius = twist.linear.x/twist.angular.z
			leftWheel  = -(twist.angular.z * (cmdRadius +b/2))*166
			rightWheel =  (twist.angular.z * (cmdRadius - b/2))*166
		else:
			leftWheel = -(twist.linear.x)*166
			rightWheel = (twist.linear.x)*166
			
		wayLeft = 0
		wayRight = 0
		
		if(leftWheel>= 0):
			wayLeft=0
		else:
			wayLeft=1
		
		if(rightWheel>= 0):
			wayRight=1
		else:
			wayRight=0

		speed1String = str(abs(int(leftWheel)))
		speed2String = str(abs(int(rightWheel)))

		if abs(leftWheel) < 10:
			speed1String = "00" + str(abs(int(leftWheel)))
			
		elif abs(leftWheel) < 100:
			speed1String = "0" + str(abs(int(leftWheel)))
			

		if abs(rightWheel) < 10:
			speed2String = "00" + str(abs(int(rightWheel)))
			
		elif abs(rightWheel) < 100:
			speed2String = "0" + str(abs(int(rightWheel)))

		
		print("S" + str(wayLeft) + str(speed1String) + "," + str(wayLeft) + str(speed1String) + "," + str(wayRight) + str(speed2String) + ","+ str(wayRight) + str(speed2String)+ "F")
	   
		pub.publish("S" + str(wayLeft) + str(speed1String) + str(wayLeft) + str(speed1String) + str(wayRight) + str(speed2String) + str(wayRight) + str(speed2String)+ "F")
	rospy.spin()

if __name__ == '__main__':
	main()
