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


 
    WPx =msg.pose.position.x
    WPy =msg.pose.position.y
    

    
    if(WPx != xold or WPy != yold):
        move_base(WPx,WPy)

    xold=WPx
    yold=WPy

      
def callback_sensor(data):

    global latCur
    global longCur
    splitted_data= data.data.split(',')
    if(splitted_data[1] !='' and splitted_data[2] !=''  ):
        latCur=(float(splitted_data[1]))
        longCur=(float(splitted_data[2]))

def callback_waypoint(data):
    global latDesired,longDesired
    splitted_waypoint = data.data.split(',')
    if(splitted_waypoint[0] != '' and splitted_waypoint[1] != ''):
        latDesired = float(splitted_waypoint[0])
        longDesired = float(splitted_waypoint[1])


def move_base(x,y):
        global checkController,WPSent, latCur, longCur,latDesired,longDesired


        sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
        goal = MoveBaseGoal()



        #rospy.loginfo("Move base started WPS = x: %s y: %s" ,str(x),str(y))
        #print("Current x : " + str(longCur) + "  Way x : " + str(longDesired))





        if WPSent <2:



            goal.target_pose.pose.position.x = float(x)
            goal.target_pose.pose.position.y = float(y)
            goal.target_pose.pose.orientation.w = 1.0
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            sac.send_goal(goal)
            WPSent += 1

        #print("Current WP : " + str(goal.target_pose.pose.position.x) + " " + str(goal.target_pose.pose.position.y))

        rospy.sleep(2.)

            #print sac.get_result()

        #if longCur > float(longDesired) - 0.00002 and longCur < float(latDesired) + 0.00002:
            #print("Here2")
           # cancelGoal()
            #cancelGoal().newGoalHandler()
            #print("DONE")







def main():
    rospy.init_node('rover_autonom1')
    rate = rospy.Rate(0.05)
    while not rospy.is_shutdown():
        rospy.Subscriber("/goal", PoseStamped, callback_dist)
        rospy.Subscriber('/rover_serial_sensor', String, callback_sensor)
        rospy.Subscriber('/desiredWayPoint',String,callback_waypoint)



        rate.sleep()




if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
