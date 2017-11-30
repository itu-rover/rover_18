#!/usr/bin/env python

import rospy
import serial
import time
from std_msgs.msg import String
from settings import * 
 
namespace = '[RoverSerial : ] '
serialMsg = ""

#get data from pc
def serialCallback(data):
    global serialMsg
    serialMsg = data.data

def portIsUsable(portName):
    try:
       ser = serial.Serial(port=portName)
       return True
    except:
       return False

def letsSerial():

    rospy.init_node("rover_serials")
    rospy.Subscriber('rover_serial_topic', String, serialCallback)

    sensor_pub = rospy.Publisher('rover_serial_sensor', String, queue_size=10)
    sensor_pub2 = rospy.Publisher('rover_serial_imu', String, queue_size=10)

    imu_flag = False
    motor_flag = False

    serialString1 = '/dev/ttyUSB' + str(MOTOR_SERIAL_PORT)
    serialString2 = '/dev/ttyUSB' + str(IMU_SERIAL_PORT)

    global namespace
    global serialMsg
    if(portIsUsable(serialString1)== False):
        print(serialString1+ " can not opened")
    else:
        print(serialString1+" is opened ")
        motor_flag=True

    if(portIsUsable(serialString2)== False):
        print(serialString2+ " can not opened")
    else:
        print(serialString2+" is opened ")
        imu_flag=True

    while not rospy.is_shutdown():


        
        if(motor_flag==True): #motor

            print(serialString1+" is opened")

            ser = serial.Serial(port=serialString1, baudrate=9600,parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)  # open serial
            ser.timeout = 1
            receive = ser.readline()
            ser.writelines(serialMsg + "\n")
            ser.flushInput()
            ser.flushOutput()
            sensor_pub.publish(receive)
            print(namespace + "I'm reading this from serial : " + str(receive) + "   I'm writing to serial :  " + serialMsg + "\n")
           

        
        if(imu_flag==True): #imu

            

            ser2 = serial.Serial(port=serialString2,  baudrate=115200, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS)  # open serial
            ser2.timeout = 1
            receive2 = ser2.readline()
            ser2.flushInput()
            ser2.flushOutput()
            sensor_pub2.publish(receive2)
           
            print(namespace + "I'm reading this from serial : " + str(receive2) + "\n")
           
           
            

        time.sleep(0.05)
    rospy.spin()



if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass