#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import threading
import numpy as np
from serial_com import *
from settings import *
import socket
# import os,system
import subprocess
import time


class CommHandler(object):
    def __init__(self):
        rospy.init_node('roverListener')
        self.client = socket.socket()
        self.client.connect((HOST,PORT))
        self.raw_data = None
        self.sending_string = "///"
        self.handler()

    def callback(self,data):

        self.raw_data = data.data
        # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', str(self.raw_data))
        self.raw_data = "T" + self.raw_data + "\n"
        self.sending_string = "B100000E" + "/" + "B1000000E" + "/" + self.raw_data


    def handler(self):
        rate = rospy.Rate(2)
        is_first = 0
        while not rospy.is_shutdown():
            # sensor_data = self.client.recv(1024)
            # sensor_data_array = sensor_data.split(',')
            # start_flag = int(sensor_data_array[3])
            start_flag = 1
            # print("Before Flag")
            if (start_flag == 1 and is_first == 0):
                print("Image Processing Started")
                subprocess.Popen(['./image_proc.sh'])
                # os.system('rosrun rover_vision test_tennis_ball')
                is_first = 1
                start_flag = 0 
            
            # print(start_flag)
            # print(is_first)
            # self.start_im_proc_pub.publish(str(self.start_flag))
            if(is_first == 1):
                rospy.Subscriber('final_vel', String, self.callback)
                self.client.send(self.sending_string)
                print str(self.sending_string)
            # self.client.send(self.raw_data)
            # print str(self.raw_data)

            rate.sleep()
            # time.sleep(0.5)
            # rospy.spin()

if __name__ == '__main__':
    CommHandler() 