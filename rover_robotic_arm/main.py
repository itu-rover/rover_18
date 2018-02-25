from robotic_arm import RoverArm
from grapher import Grapher
from ps4 import PS4Controller


ps4 = PS4Controller()
p = [40,0,40]
s = [1, 0, 0]
ps4.init()
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
    global test, lines, p, ps4
    llist = ps4.listen()[0]
    if type(llist.get(0)) != type(None):
        s[1] -= llist.get(0) / 5.
    if type(llist.get(1)) != type(None):
        s[2] -= llist.get(1) / 5.

    # if type(llist.get(0)) != type(None):
    #     p[1] -= llist.get(0) / 2.
    # if type(llist.get(1)) != type(None):
    #     p[0] -= llist.get(1) / 2.
    # if type(llist.get(3)) != type(None):
    #     p[2] -= llist.get(3) / 2.

    # p[0] += llist.get(1) / 5.


    test.update_destination_point(p, s)
    #test.print_info()
    lines = to_lines(test.joint_points)
    g.redraw(lines)

test = RoverArm([50, 40, 15])

test.update_destination_point([40,0,40], [1,0,0])
#test.print_info()
lines = to_lines(test.joint_points)

g = Grapher(lines)
g.redraw(lines)
g.show(anim)
