#!/usr/bin/env python

import rospy
import serial
import time
from std_msgs.msg import String

namespace = '[RoverSerial : ] '
serialMsg = ""



def letsSerial():
    rospy.init_node("rover_serial")
    sensor_pub = rospy.Publisher('rover_serial_imu',String,queue_size=10)

    global namespace
    global serialMsg
    printOnce = True
    while not rospy.is_shutdown():

        ser = serial.Serial(port='/dev/ttyUSB2', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS) #open serial
        ser.timeout = 1


        rospy.loginfo(ser.portstr)

        while ser.isOpen():
            if printOnce == True:

                print(namespace + "serial is open")
                printOnce = False





            receive = ser.readline()
            sensor_pub.publish(receive)
            print(namespace + "I'm reading this from serial : " + str(receive) + "\n")

            ser.flushInput()
            ser.flushOutput()

            time.sleep(0.05)
        rospy.spin()



if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass