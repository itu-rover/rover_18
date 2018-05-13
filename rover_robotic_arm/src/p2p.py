from robotic_arm import RoverArm
from grapher import Grapher
from vectorial_calculations import subtract, length, make_unit, scalar_of_vector, sum_vector
from time import sleep

lines = []
base_lines = []

def to_lines(points):
    v = []
    for i in range(0, len(points)):
        if i == 0:
            v.append([[0,0,0], points[i]])
        else:
            v.append([points[i-1], points[i]])
    return v

p = [65,-20,40]
s = [1, 1, 0]
tp = [40, 35, 60]
ts = [0, 2, 1]

arm = RoverArm([63, 47, 21])
arm.update_destination_point(tp, ts)
last_pos = to_lines(arm.joint_points)
arm.update_destination_point(p, s)
first_pos = to_lines(arm.joint_points)

def generate_curve(current_position, current_direction, target_position, target_direction, iteration_factor, k1, k2, minimum_points=3):
    p1 = sum_vector(scalar_of_vector(make_unit(current_direction), -k1), current_position)
    p2 = sum_vector(scalar_of_vector(make_unit(target_direction), -k2), target_position)
    points = []


    directions = [current_direction]
    for i in [float(j) / iteration_factor for j in range(0, iteration_factor, 1)]:
        t = i
        Bx = pow((1 - t), 3) * current_position[0] + 3 * t * pow((1 - t), 2) * p1[0] + 3 * t * t * (1 - t) * p2[0] + pow(t, 3) * target_position[0]
        By = pow((1 - t), 3) * current_position[1] + 3 * t * pow((1 - t), 2) * p1[1] + 3 * t * t * (1 - t) * p2[1] + pow(t, 3) * target_position[1]
        Bz = pow((1 - t), 3) * current_position[2] + 3 * t * pow((1 - t), 2) * p1[2] + 3 * t * t * (1 - t) * p2[2] + pow(t, 3) * target_position[2]
        points.append([Bx, By, Bz])
        dir_step_v = scalar_of_vector(subtract(target_direction, current_direction), t)
        directions.append(sum_vector(current_direction, dir_step_v))
    points.append(target_position)
    directions.append(target_direction)
    avg_step_distance = 0
    for i in range(len(points) - 1):
        avg_step_distance += abs(length(subtract(points[i + 1], points[i])))
    avg_step_distance /= len(points)
    return points, directions, avg_step_distance

def generate_path(current_position, target_position, step_dist, minimum_points=3):
    movement_vector = subtract(target_position, current_position)
    distance = length(movement_vector)
    directional_vector = make_unit(movement_vector)

    number_of_points = int(distance / step_dist)
    if number_of_points == 0:
        number_of_points = minimum_points + 2
    division_distance = distance / number_of_points

    points_on_path = []
    for i in range(0, number_of_points):
        v = [0, 0, 0]
        v = scalar_of_vector(directional_vector, division_distance * (i + 1))
        v = sum_vector(v, current_position)
        points_on_path.append(v)
    return points_on_path

safety_const = 15
safe_approach_point = sum_vector(tp, scalar_of_vector(make_unit(ts), -safety_const))
points, dirs, avg_step = generate_curve(p, s, safe_approach_point, ts, 20, 20, 20)

_points = generate_path(safe_approach_point, tp, avg_step)
for i in _points:
    points.append(i)

print "Current Position: {0}, Target Position: {1}, Approaching the target position from: {2}".format(p, tp, safe_approach_point)
print "Current Direction: {0}, Taget Direction: {1}".format(s, ts)
print "Average Step Distance: {0}".format(avg_step)
print "Total Points on Path: {0}".format(len(points))

count = 0
def anim():
    global p, s, arm, count, points, dirs, first_pos, base_lines, last_pos
    # Main Code:
    if count < len(points):
        c = count
        if count >= len(dirs):
            c = len(dirs) - 1
        arm.update_destination_point(points[count], dirs[c])
        g.redraw_point(points[count])
        count += 1
    else:
        count = 0
        g.clear_points()

    lines = to_lines(arm.joint_points)
    base_lines = first_pos
    for line in last_pos:
        base_lines.append(line)
    g.redraw(lines)
    g.redraw_base(base_lines)

    # sleep(0.01)

g = Grapher(lines)
g.redraw(lines)
g.show(anim)
