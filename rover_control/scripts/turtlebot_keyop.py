#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist




def main():
    
    rospy.init_node('cmd_publisher')
    twist =Twist();
    pub=rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=50)

    rate = rospy.Rate(10) # 10hz
   
    while not rospy.is_shutdown():
     twist.linear.x = 0.05
     pub.publish(twist)
     print(str(twist))
        

     rate.sleep()
        
  


    rospy.spin()

if __name__ == '__main__':
    main()
