#!/usr/bin/env python

__author__ = "Furkan Ayik"


from threading import Timer,Thread,Event
import numpy as np
import imutils
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import rosparam
from cv_bridge import CvBridge, CvBridgeError
import rosparam

"""
Before use !

You might be need this command for terminal  ->  'sudo rmmod peaq_wmi'

"""
coordinates_x = []
coordinates_y = []
mean_x = None
mean_y = None
pxTopic = rospy.get_param('RoverReachImage/ImageProcessing/pub_pxCoordinates','/px_coordinates')
resize = rospy.get_param('RoverReachImage/ImageProcessing/resize',False)
resize_width = rospy.get_param('RoverReachImage/ImageProcessing/resize_width',1280)
resize_height = rospy.get_param('RoverReachImage/ImageProcessing/resize_height',960)

# Low Pass Filter with a Period = 0.5 seconds
def mean_value(co_x, co_y):
	global mean_x
	global mean_y
	global coordinates_x
	global coordinates_y
	#print("-----------------------")
	#print("Length of arrays before cleaning..\nx,y : {0}--{1}".format(len(co_x),len(co_y)))

	mean_x = float(sum(co_x) / max(len(co_x), 1))
	mean_y = float(sum(co_y) / max(len(co_y), 1))

	#print("Latest Coordinates:{0}--{1}\n ".format(mean_x, mean_y))
	print("Center : {0}--{1}\n ".format(mean_x, mean_y))
	#frees arrays of coordinates
	coordinates_x = []
	coordinates_y = []
	

class LowPassFilter(Thread):
	global coordinates_x
	global coordinates_y
	def __init__(self, event):
		Thread.__init__(self)
		self.stopped = event

	def run(self):
		while not self.stopped.wait(0.1):
			# Call the function if any coordinates detected
			if len(coordinates_y) + len(coordinates_x) != 0:
				mean_value(coordinates_x,coordinates_y)


def main():
	#Coordinates
	global coordinates_x
	global coordinates_y
	coordinates_x = [0]
	coordinates_y = [0]
	global xCoordinate
	global yCoordinate
	global resize
	global resize_width, resize_height
	global mean_x
	global mean_y

	#Start Timer
	stopFlag = Event()
	thread = LowPassFilter(stopFlag)
	thread.start()

	#Video Path
	video_path = '' #Enter any video path relative to the script file

	#Treshold for Green in BGR Color Space
	greenLower = (29, 50, 150)  # Less accurate -> (29,86,6)
	greenUpper = (64, 255, 255)

	#Detection in real time
	camera = cv2.VideoCapture(1)

	#Detection over video
	#camera = cv2.VideoCapture(video_path)

	while not rospy.is_shutdown():#Use this command for detection over a video instead 'True' --> 'camera.isOpened()'


		#Read Frame
		(_, frame) = camera.read()
		height, width = frame.shape[:2]
		frame = frame[0:height,0:int(width*0.5)]
		# Resize and Add Noise
		if resize == True:
			frame = imutils.resize(frame,width = resize_width, height= resize_height)
		

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.bitwise_and(mask, mask, mask=mask)

		# Masking
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=3)
		#mask = cv2.GaussianBlur(mask, (5, 5), 0)

		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None


		#Execute only at least one contour found
		if len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			# Select contours with a size bigger than 0.1
			if radius > 0.2:
				# draw the circle and center
				cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)

				#Hold Coordinates
				coordinates_x.append(center[0])
				coordinates_y.append(center[1])
				

				#Free Coordinates if timer is up
				if set == 0:
					coordinates_x = []
					coordinates_y = []
					xCoordinate = 0
					yCoordinate = 0

				frameHeight = frame.shape[0]
				frameWidth = frame.shape[1]
				coordinatePublisher.publish(str(mean_x) +","+ str(mean_y) + "," + str(frameWidth) + "," + str(frameHeight))
		else:
			coordinatePublisher.publish("-")
				

		cv2.imshow("Frame", frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			stopFlag.set()
			break

	stopFlag.set()
	camera.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':

	try:
		rospy.init_node('rover_detect_ball')		
		coordinatePublisher = rospy.Publisher(pxTopic,String,queue_size = 1)
		while not rospy.is_shutdown():
			main()
	except rospy.ROSInterruptException:
		pass