#!/usr/bin/env python
#This is the Modelling Code  for ITU Rover Team
##This code takes pictures with pressing space bar and mark the gps data to their exif's.

import numpy as np
import imutils
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

def main():
	currentMode = "Modelling"

	while not rospy.is_shutdown():
		print("Enter: Take Picture \n ")
		print("0: Change mode, current Mode : " + str(currentMode))
		userInput = raw_input()
		if userInput == "":
			if currentMode == "Modelling":
				controlPub.publish("1")
			elif currentMode == "Panaromic":
				controlPub.publish("2")
		elif userInput == "0":
			controlPub.publish("2")
			if currentMode == "Modelling":
				currentMode = "Panaromic"
			elif currentMode == "Panaromic":
				currentMode = "Modelling"


if __name__ == '__main__':

	try:
		rospy.init_node('rover_modelling_controller')
		controlPub = rospy.Publisher('/rover_modelling/control',String,queue_size=100)
		while not rospy.is_shutdown():
			main()
	except rospy.ROSInterruptException:
		pass