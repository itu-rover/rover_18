#!/usr/bin/env python


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

bridge = CvBridge()

def main():
	
	camera = cv2.VideoCapture(1)

	#Detection over video
	#camera = cv2.VideoCapture(video_path)

	while not rospy.is_shutdown():
		(_, frame) = camera.read()
		img = Image()
		#height, width = frame.shape[:2]
		img = bridge.cv2_to_imgmsg(frame,"bgr8")
		imagePublisher.publish(img)

	camera.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':

	try:
		rospy.init_node('send_image')		
		imagePublisher = rospy.Publisher("/image_elevation",Image,queue_size = 10)
		while not rospy.is_shutdown():
			main()
	except rospy.ROSInterruptException:
		pass
