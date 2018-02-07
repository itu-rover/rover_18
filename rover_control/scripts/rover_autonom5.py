#!/usr/bin/env python

import roslib
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geographic_msgs.msg import WayPoint
#move_base_msgs
from move_base_msgs.msg import *
import math
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
from nav_msgs.msg import Odometry
import actionlib_tutorials.msg
import actionlib_msgs.msg



splitted_data = 0
WPSent = 0
checkController = 0
latCur =0
longCur = 0
latDesired = 0
longDesired = 0
xold=0
yold=0

WPSent=0

class cancelGoal():
    def __init__(self):

        self.explore_goal_sub = rospy.Subscriber("/move_base/goal",
                                    MoveBaseActionGoal, self.newGoalHandler)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base server.....")
        self.client.wait_for_server()



    def newGoalHandler(self):
        self.client.cancel_goal()
        self.client.cancel_all_goals() # Try both
        rospy.loginfo("Goal cancelled")


def callback_dist(msg):


    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    
    
    
     
    desiredPose = PoseStamped()
    desiredPose.header.frame_id = "/odom"


    desiredPose.pose.position.x = x
    desiredPose.pose.position.y = y
    desiredPose.pose.position.z = 0
    desiredPose.pose.orientation.x = 0
    desiredPose.pose.orientation.y = 0
    desiredPose.pose.orientation.z = 0
    desiredPose.pose.orientation.w = 1
    
    if(xold != x or yold !=y ):
        navGoalPub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10) #Publish Nav Goal to ROS topic
        navGoalPub.publish(desiredPose)

    


      
def callback_sensor(data):

    global latCur
    global longCur
    splitted_data= data.data.split(',')
    if(splitted_data[1] !='' and splitted_data[2] !=''  ):
        latCur=(float(splitted_data[1]))
        longCur=(float(splitted_data[2]))
    

def main():
    rospy.init_node('simple_move')
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        

       rospy.Subscriber("/odometry/goal",Odometry, callback_dist)

       rospy.Subscriber('/rover_serial_sensor',String, callback_sensor)
        
       

        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
