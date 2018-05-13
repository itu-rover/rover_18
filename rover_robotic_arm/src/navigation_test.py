from robotic_arm import RoverArm
from navigation import Navigation
from grapher import Grapher

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

arm = RoverArm([63, 47, 21])
arm.update_destination_point(p, s)


def anim():
    global p, s, arm, nav
    inp = int(raw_input("zoom?"))
    nav.zoom(inp)
    p = nav.position
    arm.update_destination_point(p, s)
    lines = to_lines(arm.joint_points)
    g.redraw(lines)


g = Grapher(lines)
g.redraw(lines)
g.show(anim)
