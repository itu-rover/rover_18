#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    
    sensor_data=data.data
    sensor_split = sensor_data.split(',')
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", sensor_split[2])

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("rover_serial_sensor", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
