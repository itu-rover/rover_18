from robotic_arm import RoverArm
from grapher import Grapher
from dualshock4 import PS4Controller
from robotic_arm import length
import time

controller = PS4Controller(0x046d, 0xc215)
k = 0.005
p = [65,0,40]
s = [1, 0, 0]
lines = []

def to_lines(points):
    v = []
    for i in range(0, len(points)):
        if i == 0:
            v.append([[0,0,0], points[i]])
        else:
            v.append([points[i-1], points[i]])
    return v

def anim():
    global controller, lines, p, s, test
    print length(test.joint_points[1])
    controller.update()
    p[0] += k * (controller.raw_data[3] - 128)
    p[2] += k * (controller.raw_data[5] - 128) * -1
    test.update_destination_point(p, s)
    #test.print_info()
    lines = to_lines(test.joint_points)
    g.redraw(lines)
    time.sleep(0.02)

test = RoverArm([63, 47, 21])

test.update_destination_point([65,0,40], [1,0,0])
#test.print_info()
lines = to_lines(test.joint_points)

g = Grapher(lines)
g.redraw(lines)
g.show(anim)
