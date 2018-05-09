#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf

class GoForwardAvoid():
    def __init__(self):
        rospy.init_node('nav_test3', anonymous=False)

        rate = rospy.Rate(10) # 10hz
        #tell the action client that we want to spin a thread by default
        count=0

        while not rospy.is_shutdown():

            GoalPub = rospy.Publisher('/bearing_to_ball', String, queue_size=10) #Publish Nav Goal to ROS topic
            print(count)  
            if(count>=100):
              GoalPub.publish("-")
        
            if(count<100):
               a=5
               GoalPub.publish(str(a))
               count=count+1

               
            rate.sleep()

    
    

if __name__ == '__main__':
    try:
        GoForwardAvoid()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")