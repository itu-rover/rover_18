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
    rospy.Subscriber('rover_serial_topic', String, serialCallback)
    sensor_pub = rospy.Publisher('rover_serial_sensor', String, queue_size=10)
    sensor_pub2 = rospy.Publisher('rover_serial_imu', String, queue_size=10)

    global namespace
    global serialMsg
    printOnce = True
    while not rospy.is_shutdown():

        serString1 = raw_input("Enter First Port: ")


        if serString1 != 'n':

            serialString1 = '/dev/ttyUSB' + str(serString1)
            print(serialString1)
            ser = serial.Serial(port=serialString1, baudrate=115200, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)  # open serial
            ser.timeout = 1

            serString2 = raw_input("Enter Second  Port: ")


            if serString2 == 'n':
                while ser.isOpen():
                    if printOnce == True:
                        print(namespace + "serial1 is open")
                        printOnce = False


                    receive = ser.readline()
                    sensor_pub.publish(receive)
                    print(namespace + "I'm reading this from serial : " + str(receive) + "   I'm writing to serial :  " + serialMsg + "\n")
                    ser.flushInput()
                    ser.flushOutput()
                    time.sleep(0.05)
                rospy.spin()


            else:
                serialString2 = '/dev/ttyUSB' + str(serString2)
                print(serialString2)
                ser2 = serial.Serial(port=serialString2, baudrate=9600, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)  # open serial
                ser2.timeout = 1


                while ser.isOpen() and ser2.isOpen():
                    if printOnce == True:
                        print(namespace + "serial1  and serial 2 is open")
                        printOnce = False

                    receive = ser.readline()
                    receive2 = ser2.readline()
                    sensor_pub.publish(receive)
                    sensor_pub2.publish(receive2)
                    print(namespace + "I'm reading this from serial : " + str(receive) + "   I'm writing to serial :  " + serialMsg + "\n")
                    print(namespace + "I'm reading this from serial : " + str(receive2) + "\n")

                    ser.flushInput()
                    ser.flushOutput()
                    ser2.flushInput()
                    ser2.flushOutput()

                    time.sleep(0.05)
                rospy.spin()







if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass