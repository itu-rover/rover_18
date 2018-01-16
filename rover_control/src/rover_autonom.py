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
from rover_control.msg import Distancewithangle
import math
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2


xold=0
yold=0

def callback_dist(msg):

    global xold
    global yold

    incoming_data=Distancewithangle()
    incoming_data.currPosX =msg.currPosX
    incoming_data.currPosY =msg.currPosY
    incoming_data.distance=msg.distance
    incoming_data.angle=msg.angle


    
    
    x= float (incoming_data.currPosX ) +( float(incoming_data.distance) * cos(float(incoming_data.angle)))
    y= float (incoming_data.currPosY) +( float(incoming_data.distance) * sin(float(incoming_data.angle)))
    if(x != xold or y != yold):
        move_base(x,y)

    xold=x
    yold=y


    
      


def move_base(x,y):

        sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
        goal = MoveBaseGoal()
        rospy.loginfo("Move base started x: %s y: %s" ,str(x),str(y))

        goal.target_pose.pose.position.x =  float(x)
        goal.target_pose.pose.position.y =  float(y)
        goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()

        #start listner
        sac.wait_for_server()

         #send goal
        sac.send_goal(goal)

        #finish
        sac.wait_for_result()
        print sac.get_result()

def main():
    rospy.init_node('simple_move')
    rate = rospy.Rate(0.05)
    while not rospy.is_shutdown():
    	rospy.Subscriber("/distancewithangle", Distancewithangle, callback_dist)
    	rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
