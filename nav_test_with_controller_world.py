from robotic_arm import RoverArm
from navigation import Navigation
from grapher import Grapher
from ps4controller.ps4 import PS4Controller
from ps4controller.ps4 import add_buttons
from vectorial_calculations import cross
import time

lines = []
ghost_lines = []
global_speed_gain = 1.0
loop_rate = 1.0
def to_lines(points):
    v = []
    for i in range(0, len(points)):
        if i == 0:
            v.append([[0, 0, 0], points[i]])
        else:
            v.append([points[i - 1], points[i]])
    return v

initial_p = [85, 0, 40]
initial_s = [1, 0, 0]
p = [85, 0, 40]
s = [1, 0, 0]
nav = Navigation(p, s)
ps4 = PS4Controller(0x054C, 0x09CC)
arm = RoverArm([63, 47, 21])
arm.update_destination_point(p, s)
# arm.establish_serial_connection()
arm.establish_tcp()

p_temp = [0, 0, 0]
s_temp = [0, 0, 0]

GRAPH = False

def anim():
    global p, s, arm, nav, global_speed_gain, loop_rate, ghost_lines
    ps4.update()

    up_down_vel = (ps4.axis_raw[5] - ps4.axis_raw[4]) / 255.0
    rl_vel = -(ps4.axis_raw[0] - 128) / 128.0
    zoom_vel = ((256 - ps4.axis_raw[1]) - 128) / 128.0

    yaw_vel = (ps4.axis_raw[2] - 128) / 128.0
    pitch_vel = ((256 - ps4.axis_raw[3]) - 128) / 128.0

    ud_normal = cross(cross(arm.vectors[1], arm.vectors[2]), arm.vectors[2])
    rl_normal = cross(arm.vectors[2], [0, 0, 1])
    pitch_normal = cross(arm.vectors[2], arm.vectors[1])

    nav.rotate_yaw(ud_normal, yaw_vel)
    nav.rotate_pitch(pitch_normal, pitch_vel)
    # inp = int(raw_input("zoom?"))
    # nav.zoom(inp)
    # p_temp[0] = p[0]
    # p_temp[1] = p[1]
    # p_temp[2] = p[2]
    #
    # s_temp[0] = p[0]
    # s_temp[1] = p[1]
    # s_temp[2] = p[2]

    p[2] += up_down_vel * (1.0 / loop_rate) * global_speed_gain
    p[1] += rl_vel * (1.0 / loop_rate) * global_speed_gain
    p[0] += zoom_vel * (1.0 / loop_rate) * global_speed_gain
    s = nav.vector

    if ps4.additional_buttons == 1:
        p[0] = initial_p[0]
        p[1] = initial_p[1]
        p[2] = initial_p[2]
        s[0] = initial_s[0]
        s[1] = initial_s[1]
        s[2] = initial_s[2]

    print "[ SERIAL ] " + arm.return_model_for_low_level()
    print "[ ANGLES ] " + str(arm.joint_angles[0]), arm.joint_angles[1], 180 - arm.joint_angles[2], arm.joint_angles[3], arm.joint_angles[4]
    print "[ OK ] " + str(p) , str(s)

    (limit_state, ghost_points) = arm.update_destination_point(p, s)

    if not limit_state and not ghost_points is None:
        ghost_lines = to_lines(ghost_points)
    else:
        ghost_lines = []
    # arm.serial_write()
    arm.tcp_write()
    lines = to_lines(arm.joint_points)
    # g.redraw_point(arm.joint_points[1])
    if GRAPH:
        g.redraw(lines)
        g.redraw_base(ghost_lines)

if GRAPH:
    g = Grapher(lines)
    t_point1 = [50, 10, 40]
    t_point2 = [70, 5, 30]
    g.redraw_point(t_point1)
    g.redraw_point(t_point2)
    g.redraw(lines)
    g.show(anim)
else:
    while True:
        anim()
        time.sleep(0.01)
