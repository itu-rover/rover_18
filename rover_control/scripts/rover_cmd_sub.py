#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


twist =Twist();
pub=rospy.Publisher("/rover_serial_topic", String, queue_size=50)
def callbackcmd(data):
    #taking the data from husky_controller/cmd_vel  topic
    twist.linear.x = data.linear.x 
    twist.angular.z = data.angular.z
    #if anglular speed is negative angular way is 0 else 1 
    if twist.angular.z < 0:
        angular_way = "1"
        rover_angular_speed = -twist.angular.z

    else:
        angular_way = "0"
        rover_angular_speed = twist.angular.z
    #if linear speed is negative angular way is 0 else 1 
    if twist.linear.x< 0:
        linear_way = "0"
        rover_linear_speed = -twist.linear.x

    else:
        linear_way = "1"
        rover_linear_speed = twist.linear.x
        # Linear Speed is translating to string
    rover_linear_speed = int(rover_linear_speed*99/1)

    if rover_linear_speed < 10:
        rover_linear_speed_str = "0" + str(rover_linear_speed)

    else:
        rover_linear_speed_str = str(rover_linear_speed)

        # Angular Speed is translating to string
    rover_angular_speed = int(rover_angular_speed*99/1)

    if rover_angular_speed < 10:
        rover_angular_speed_str = "0" + str(rover_angular_speed)
    else:
        rover_angular_speed_str = str(rover_angular_speed)

   

    
    #rospy.loginfo( "I heard %s", "M"+ linear_way + rover_linear_speed_str + angular_way+rover_angular_speed_str+"E")
   
    pub.publish('M'+ linear_way + rover_linear_speed_str + angular_way + rover_angular_speed_str +'E')

def main():

    rospy.init_node('rover_cmd_sub_serial')
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, callbackcmd)
    #rospy.Subscriber("/rover_joy/cmd_vel", Twist, callbackcmd)
    rospy.spin()

if __name__ == '__main__':
    main()
