#!/usr/bin/env python
import rospy
from std_msgs.msg import String



def main():
    
    rospy.init_node('choose_duty')
    choose_pub = rospy.Publisher('choose_duty',String,queue_size=100)
    rate = rospy.Rate(100)
    isSent = False
    
    
    
    while not rospy.is_shutdown():
        
        chooseString = raw_input("1 for A, 2 for TT , 3 for EQ, 4 for SC , 0 for EXIT \n ")
        
        while isSent is False :
                choose_pub.publish(chooseString)
                rate.sleep()
                isSent = True
        
            


    rospy.spin()

if __name__ == '__main__':
    main()