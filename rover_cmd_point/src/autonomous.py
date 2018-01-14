#!/usr/bin/env python
import roslib
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import *



def call_back_autonomous(data):

    autonomous_pub = rospy.Publisher('autonomous',String,queue_size=10)   
    autonomousVariable = data.data
    flag=1 
    print("var:")
    print(autonomousVariable)
    if autonomousVariable =='1'and flag =='1':
        print("It is working autonomously")
        simple_move()
        print("It is a")
        autonomous_pub.publish('1')
        autonomousVariable=0
        

    
def simple_move():
    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
    #create goal
    goal = MoveBaseGoal()
    #use self?
    #set goal
    goal.target_pose.pose.position.x = 0.0
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.header.frame_id = 'odom'
    goal.target_pose.header.stamp = rospy.Time.now()
    #start listner
    sac.wait_for_server()
    #send goal
    sac.send_goal(goal)

    #finish
    sac.wait_for_result()

    #print result
    print sac.get_result()

def main():    
    
    rospy.init_node('autonomous')
    rospy.Subscriber('autonomous',String,call_back_autonomous)
    rospy.spin()

if __name__ == '__main__':
    main()
