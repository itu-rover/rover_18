#!/usr/bin/env python

import rospy
import serial
import time
from std_msgs.msg import String

namespace = '[RoverSerial : ] '
serialMsg = ""

#get data from pc
def serialCallback(data):
    global serialMsg
    #print("I heard : " + data.data)
    serialMsg = data.data


def letsSerial():
    rospy.init_node("rover_serial")
    rospy.Subscriber('rover_serial_topic',String,serialCallback)
    global namespace
    global serialMsg
    while not rospy.is_shutdown():
        ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS) #open serial

        print(ser.portstr)
    #Send the message taken from pc , to the stm through serial

        while ser.isOpen():
            print(namespace + "serial is open")
            ser.writelines(serialMsg + "\n")
            ser.flushInput()
            ser.flushOutput()
            print("I wrote : " + serialMsg)
            time.sleep(0.8)
        rospy.spin()







if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass
