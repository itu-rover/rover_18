#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#1-3 are the left side of the rover, 2-4 are the right side of the rover.

twist =Twist();
pub=rospy.Publisher("/rover_serial_topic", String, queue_size=50)
def callbackcmd(data):
	#taking the data from husky_controller/cmd_vel  topic
	if abs(data.linear.x) > 0.1:
		twist.linear.x = data.linear.x 
	else: 
		twist.linear.x = 0


	
	twist.angular.z = data.angular.z

	
		
	way1 = 0
	way2 = 0
		
	speed1 = 0
	speed2 = 0
		
		
	

	twist.linear.x = int(twist.linear.x * 375)
	twist.angular.z = int(twist.angular.z * 375)



	if twist.linear.x == 0:
		if twist.angular.z == 0:
			way1 = 0
			way2 = 0
			
			speed1 = 0
			speed2 = 0
			

		elif twist.angular.z < 0:
			way1 = 0  
			way2 = 1
			
			speed1 = -1 * twist.angular.z    		
			speed2 = -1* twist.angular.z
			

		elif twist.angular.z > 0:
			way1 = 1    		
			way2 = 0
			
			speed1 = twist.angular.z    		
			speed2 = twist.angular.z
			


	elif twist.linear.x > 0:
		if twist.angular.z == 0:
			way1 = 1
			way2 = 1
			
			speed1 = twist.linear.x
			speed2 = twist.linear.x
			

		elif twist.angular.z < 0 :
			way1 = 1
			way2 = 1
			
			if abs(twist.linear.x) >  abs(twist.angular.z):
				speed1 = twist.linear.x + twist.angular.z
				speed2 = twist.linear.x
				
			else:
				
				speed1 = twist.linear.x
				speed2 = twist.linear.x - twist.angular.z
			

		elif twist.angular.z > 0:
			way1 = 1
			way2 = 1
			
			
			
			if abs(twist.linear.x) >  abs(twist.angular.z):
				speed2 = twist.linear.x - twist.angular.z
				speed1 = twist.linear.x

				
			else:
				speed2 = twist.linear.x
				speed1 = twist.linear.x + twist.angular.z
				


	elif twist.linear.x < 0:
		if twist.angular.z == 0:
			way1 = 0
			way2 = 0
			way3 = 0
			way4 = 0
			speed1 = -1*twist.linear.x
			speed2 = -1*twist.linear.x
			

		elif twist.angular.z > 0:
			way1 = 0
			way2 = 0
			
			
			
			if abs(twist.linear.x) >  abs(twist.angular.z):
				speed1 = -1* twist.linear.x
				speed2 = -1* twist.linear.x - twist.angular.z
				
			else:
				speed1 = -1* twist.linear.x + twist.angular.z
				speed2 = -1* twist.linear.x
				

		elif twist.angular.z < 0:
			way1 = 0
			way2 = 0
			
			if abs(twist.linear.x) >  abs(twist.angular.z):
				speed1 = -1*twist.linear.x + twist.angular.z
				speed2 = -1*twist.linear.x
				
			else:
				speed1 = -1*twist.linear.x
				speed2 = -1*twist.linear.x - twist.angular.z 
				
			
	
	speed1 = int(speed1/2)
	speed2 = int(speed2/2)
	speed1String = str(int(speed1))
	speed2String = str(int(speed2))	

	if speed1 < 10:
		speed1String = "00" + str(speed1)
		
	elif speed1 < 100:
		speed1String = "0" + str(speed1)
		

	if speed2 < 10:
		speed2String = "00" + str(speed2)
		
	elif speed2 < 100:
		speed2String = "0" + str(speed2)




 
	

   

	
	print("S" + str(way1) + str(speed1String) + "," + str(way1) + str(speed1String) + "," + str(way2) + str(speed2String) + ","+ str(way2) + str(speed2String)+ "F")
   
	pub.publish("S" + str(way1) + str(speed1String) + str(way1) + str(speed1String) + str(way2) + str(speed2String) + str(way2) + str(speed2String)+ "F")

def main():

	rospy.init_node('rover_cmd_sub_serial')
	rospy.Subscriber("/rover_joy/cmd_vel", Twist, callbackcmd)
	rospy.Subscriber("/joy_teleop/cmd_vel", Twist, callbackcmd)
	#rospy.Subscriber("/rover_joy/cmd_vel", Twist, callbackcmd)
	rospy.spin()

if __name__ == '__main__':
	main()
