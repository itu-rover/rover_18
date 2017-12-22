#!/usr/bin python
import rospy
from std_msgs import String
from robotic_arm import RoverArm

arm = RoverArm([50, 40, 15])

# NOTE: msg format = "x,y,z:x1,y1,z1"
def target_callback(data):
    global arm
    target_point = data.split(':')[0].split(',')
    approaching_vector = data.split(':')[1].split(',')
    arm.update_destination_point(target_point, approaching_vector)
    print arm.return_model

def main():
    rospy.init("robotic_arm_ros")
    rospy.Subscriber("robotic_arm_target", String, target_callback)



if __name__ == '__main__':
    main()
