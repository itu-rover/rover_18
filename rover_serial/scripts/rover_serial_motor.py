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
    sensor_pub = rospy.Publisher('rover_serial_sensor',String,queue_size=10)

    global namespace
    global serialMsg
    printOnce = True
    while not rospy.is_shutdown():

        ser = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS) #open serial
        ser.timeout = 1

        rospy.loginfo(ser.portstr)

        while ser.isOpen():
            if printOnce == True:
                print(namespace + "serial is open")
                printOnce = False


            ser.writelines(serialMsg + "\n")


            receive = ser.readline()
            sensor_pub.publish(receive)
            print(namespace + "I'm reading this from serial : " + str(receive) + "   I'm writing to serial :  "  + serialMsg)

            ser.flushInput()
            ser.flushOutput()

            time.sleep(0.05)
        rospy.spin()



if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass