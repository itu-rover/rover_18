from robotic_arm import RoverArm
from navigation import Navigation
from grapher import Grapher
from ps4controller.ps4 import PS4Controller
from vectorial_calculations import cross
lines = []


def to_lines(points):
    v = []
    for i in range(0, len(points)):
        if i == 0:
            v.append([[0, 0, 0], points[i]])
        else:
            v.append([points[i - 1], points[i]])
    return v


p = [65, 0, 40]
s = [1, 1, 0]
nav = Navigation(p, s)
ps4 = PS4Controller(0x054C, 0x09CC)
arm = RoverArm([63, 47, 21])
arm.update_destination_point(p, s)




def anim():
    global p, s, arm, nav
    ps4.update()
    up_down_vel = (ps4.axis_raw[5] - ps4.axis_raw[4]) / 255.0
    rl_vel = (ps4.axis_raw[0] - 128) / 128.0
    zoom_vel = ((255 - ps4.axis_raw[1]) - 128) / 128.0

    yaw_vel = (ps4.axis_raw[2] - 128) / 128.0
    pitch_vel = ((255 - ps4.axis_raw[3]) - 128) / 128.0
    ud_normal = cross(cross(arm.vectors[1], arm.vectors[2]), arm.vectors[2])
    rl_normal = cross(arm.vectors[2], [0, 0, 1])
    pitch_normal = cross(arm.vectors[2], arm.vectors[1])
    nav.up_down([0, 0, 1], up_down_vel)
    nav.rigth_left(rl_normal, rl_vel)
    nav.zoom(zoom_vel)
    nav.rotate_yaw(ud_normal, yaw_vel)
    nav.rotate_pitch(pitch_normal, pitch_vel)
    # inp = int(raw_input("zoom?"))
    # nav.zoom(inp)
    p = nav.position
    s = nav.vector


    arm.update_destination_point(p, s)
    lines = to_lines(arm.joint_points)
    # g.redraw_point(arm.joint_points[1])
    g.redraw(lines)


g = Grapher(lines)
t_point1 = [50, 10, 40]
t_point2 = [70, 5, 30]
g.redraw_point(t_point1)
g.redraw_point(t_point2)
g.redraw(lines)
g.show(anim)
