from robotic_arm import RoverArm
from robotic_arm import cross, make_unit
from grapher import Grapher
from math import cos, sin
import sys

r = 10
x = 0.
y = 0.
t = 0
def updatexy(_t):
    global x,y,r
    x = r * cos(_t)
    y = r * sin(_t)

p = [40,0,40]
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
    global test, lines, p, s, x, y, t, r
    t += 0.1
    updatexy(t)
    p[2] = y + 40
    p[1] = x
    test.update_destination_point(p, s)
    lines = to_lines(test.joint_points)
    g.redraw(lines)
#    sys.stdout.write("\r" + test.return_model())
    print test.return_model_for_low_level()
    sys.stdout.flush()

test = RoverArm([50, 40, 15])
test.update_destination_point([40,0,40], [1,0,0])
lines = to_lines(test.joint_points)

test.establish_serial_connection()
test.serial_write()

g = Grapher(lines)
g.redraw(lines)
g.show(anim)
