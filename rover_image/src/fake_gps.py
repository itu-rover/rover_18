#!/usr/bin/env python
#This is the Modelling Code  for ITU Rover Team
##This code takes pictures with pressing space bar and mark the gps data to their exif's.
###This code is the primary code for modelling and scaling for science task that will be done on another operating system.



import numpy as np
import imutils
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

def main():

	navMsg = NavSatFix()
	navMsg.latitude =41.2323
	navMsg.longitude = 31.2323
	while not rospy.is_shutdown():
		gpsPub.publish(navMsg)


if __name__ == '__main__':

	try:
		rospy.init_node('fake_gps')
		gpsPub = rospy.Publisher('/gps/fix',NavSatFix,queue_size=100)
		while not rospy.is_shutdown():
			main()
	except rospy.ROSInterruptException:
		pass