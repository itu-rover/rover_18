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


xold=0
yold=0
splitted_data = 0
WPSent = 0
checkController = 0
latCur =0
longCur = 0
latDesired = 0
longDesired = 0
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

   
    global xold
    global yold

    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    
    
    
     
    #if(x != xold or y != yold):
    move_base(x,y)
    

    xold=x
    yold=y

def callback_waypoint(data):
    global latDesired,longDesired
    splitted_waypoint = data.data.split(',')
    if(splitted_waypoint[0] != '' and splitted_waypoint[1] != ''):
        latDesired = float(splitted_waypoint[0])
        longDesired = float(splitted_waypoint[1])

      
def callback_sensor(data):

    global latCur
    global longCur
    splitted_data= data.data.split(',')
    if(splitted_data[1] !='' and splitted_data[2] !=''  ):
        latCur=(float(splitted_data[1]))
        longCur=(float(splitted_data[2]))
    



    
      


def move_base(x,y):

        
        global checkController,WPSent, latCur, longCur,latDesired,longDesired


        sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
        goal = MoveBaseGoal()



        rospy.loginfo("Move base started WPS = x: %s y: %s" ,str(x),str(y))
        #print("Current x : " + str(longCur) + "  Way x : " + str(longDesired))





        if WPSent <2:
            goal.target_pose.pose.position.x = float(x)
            goal.target_pose.pose.position.y = float(y)
            goal.target_pose.pose.orientation.w = 1.0
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            sac.send_goal(goal)
            WPSent = WPSent + 1

        #print("Current WP : " + str(goal.target_pose.pose.position.x) + " " + str(goal.target_pose.pose.position.y))

        rospy.sleep(4.)

            

        if longCur > float(longDesired) - 0.00002 and longCur < float(longDesired) + 0.00002:
            cancelGoal()
            cancelGoal().newGoalHandler()
            print("DONE")

def main():
    rospy.init_node('simple_move')
    rate = rospy.Rate(0.05)
    rospy.Subscriber("/odometry/goal",Odometry, callback_dist)
    rospy.Subscriber('/desiredWayPoint',String,callback_waypoint)
    rospy.Subscriber('/rover_serial_sensor',String, callback_sensor)
    while not rospy.is_shutdown():
        
        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
