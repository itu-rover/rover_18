from robotic_arm import RoverArm
from ps4controller.ps4 import PS4Controller
from vectorial_calculations import cross
from time import sleep
from navigation import Navigation

global_speed_gain = 1.0

loop_rate = 10
p = [65, 0, 40]
s = [1, 1, 0]
nav = Navigation(p, s)
ps4 = PS4Controller(0x054C, 0x09CC)
arm = RoverArm([63, 47, 21])
arm.update_destination_point(p, s)
# arm.ros_begin()
# arm.send_serial()

def anim():
    global p, s, arm, nav, global_speed_gain
    ps4.update()
    up_down_vel = (ps4.axis_raw[5] - ps4.axis_raw[4]) / 255.0
    rl_vel = (ps4.axis_raw[0] - 128) / 128.0
    zoom_vel = ((255 - ps4.axis_raw[1]) - 128) / 128.0

    yaw_vel = (ps4.axis_raw[2] - 128) / 128.0
    pitch_vel = ((255 - ps4.axis_raw[3]) - 128) / 128.0

    ud_normal = cross(cross(arm.vectors[1], arm.vectors[2]), arm.vectors[2])
    rl_normal = cross(arm.vectors[2], [0, 0, 1])
    pitch_normal = cross(arm.vectors[2], arm.vectors[1])
    nav.rotate_yaw(ud_normal, yaw_vel)
    nav.rotate_pitch(pitch_normal, pitch_vel)

    p[2] += up_down_vel * (1.0 / loop_rate) * global_speed_gain
    p[1] += rl_vel * (1.0 / loop_rate) * global_speed_gain
    p[0] += zoom_vel * (1.0 / loop_rate) * global_speed_gain

    s = nav.vector
    # print arm.joint_angles
    arm.update_destination_point(p, s)
    print arm.return_model_for_low_level()
    # arm.send_serial()

while True:
    anim()
    sleep(1.0 / loop_rate)

# while not arm.my_rospy.is_shutdown():
    # anim()
    # sleep(1.0 / loop_rate)
