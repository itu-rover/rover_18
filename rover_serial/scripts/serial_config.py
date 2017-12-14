#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time


def talker():
    sub_topic1 = 'rover_serial_topic'
    sub_topic2 =  'rover_serial_2'
    sub_topic3 = 'rover_serial_3'

    pub_topic1 = 'rover_serial_sensor'
    pub_topic2 = 'rover_serial_imu'
    pub_topic3 = 'rover_serial_sensor3'

    baudrate1 = '9600'
    baudrate2 = '115200'
    baudrate3 = '0'

    serialPort1 = '/dev/ttyUSB0'
    serialPort2 = '/dev/ttyUSB1'
    serialPort3 = '/dev/ttyUSB'


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
