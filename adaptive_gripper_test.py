from robotic_arm import RoverArm
from navigation import Navigation
from grapher import Grapher
from vectorial_calculations import cross
import time
from fixed_destinations import AdaptiveGripperClient

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


def anim(_ghosts=[]):
    global p, s, arm, nav, global_speed_gain, loop_rate, ghost_lines, g
    lines = to_lines(arm.joint_points)
    # g.redraw_point(arm.joint_points[1])
    g.redraw(lines)
    g.redraw_base(_ghosts)

p = [100, 0, 30]
s = [1, 0, 0]
nav = Navigation(p, s)
arm = RoverArm([63, 47, 21])


g = Grapher(lines)

arm.update_destination_point(p, s)
arm.adaptive1 = AdaptiveGripperClient(arm, "GRIP1", anim)
# arm.establish_serial_connection()

# while True:
#     anim()
#     time.sleep(0.1)
#
t_point1 = [50, 10, 40]
t_point2 = [70, 5, 30]
g.redraw_point(t_point1)
g.redraw_point(t_point2)
g.redraw(lines)
g.show(anim)
