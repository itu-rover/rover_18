import math
import numpy as np
import rospy

# NOTE: These functions can be added to a library - optional
# Vectorial Calculations
def RotZ(point, theta, is_theta_degrees = False):
    if (is_theta_degrees):
        theta_in_rad = math.radians(theta)
    else:
        theta_in_rad = theta

    rotZ_matrix = [ [math.cos(theta_in_rad), -math.sin(theta_in_rad), 0],
                    [math.sin(theta_in_rad), math.cos(theta_in_rad), 0],
                    [0, 0, 1]]
    result = [0, 0, 0]

    for i in range(0, 3):
        _sum = 0

        for j in range(0, 3):
            _sum += point[j] * rotZ_matrix[i][j]
        result[i] = _sum

        # # 0.0001 is the maximum error
        # if (math.fabs(result[i]) < 0.0001):
        #     result[i] = 0
    return result
def rotation_u(point_to_be_rotated, vector_rotating_around, angles_in_deg):
    u = vector_rotating_around
    p = point_to_be_rotated
    a = math.radians(angles_in_deg) # Convert to radians
    omc = 1 - math.cos(a)

    matrix = [
    [(math.cos(a) + u[0] * u[0] * omc), (u[0] * u[1] * omc - u[2] * math.sin(a)), (u[0] * u[2] * omc + u[1] * math.sin(a))],
    [(u[1] * u[0] * omc + u[2] * math.sin(a)), (math.cos(a) + u[1] * u[1] * omc), (u[1] * u[2] * omc - u[0] * math.sin(a))],
    [(u[0] * u[2] * omc - u[1] * math.sin(a)), (u[1] * u[2] * omc + u[0] * math.sin(a)), (math.cos(a) + u[2] * u[2] * omc)]
    ]

    result = [0, 0, 0]

    for i in range(0, 3):
        _sum = 0

        for j in range(0, 3):
            _sum += p[j] * matrix[i][j]
        result[i] = _sum

        # # 0.0001 is the maximum error
        # if (math.fabs(result[i]) < 0.0001):
        #     result[i] = 0

    return result
def cross(a, b):
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]
    return c
def angle(v1, v2, acute = True):
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    if (acute == True):
        return angle
    else:
        return 2 * np.pi - angle
def length(vector):
    return math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2) + math.pow(vector[2], 2))
def make_unit(vector):
    len_v = length(vector)
    return [vector[0] / len_v, vector[1] / len_v, vector[2] / len_v]
def scalar_of_vector(vector, scalar):
    return [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
def subtract(vector1, vector2):
    return [vector1[0] - vector2[0], vector1[1] - vector2[1], vector1[2] - vector2[2]]

# Triangle Calculations
def cosine_rule(c, a, b):
    cos_theta = (-1 * ((c * c) - (b * b) - (a * a))) / (2 * a * b)
    return math.acos(cos_theta)
def get_triangle_angles(a, b, c):
    return [cosine_rule(a, b, c), cosine_rule(b, a, c), cosine_rule(c, a, b)]

# Inverse Calculations
def geometric_approach(lengths, point):
    dist = length(point)
    angles = get_triangle_angles(lengths[0], lengths[1], dist)
    vector_plane_angle = math.asin(point[2] / dist)
    # To degrees Conversion
    vector_plane_angle = math.degrees(vector_plane_angle)
    for i in range(0, 3):
        angles[i] = math.degrees(angles[i])
    base_angle = vector_plane_angle + angles[1]
    joint2_angle = 180 - angles[2]
    base_rotation = math.degrees(math.atan2(point[1], point[0]))
    return [base_rotation, base_angle, joint2_angle]

class RoverArm(object):
    def __init__(self, lengths):
        self.Lengths = lengths;

    def update_destination_point(self, point, vector):
        if not length(vector) == 1:
            vector = make_unit(vector)
        last_joint_vector = scalar_of_vector(vector, self.Lengths[2])
        self.destination_point = point
        self.secondary_dest_point = subtract(point, last_joint_vector)
        self.direction_vector = vector
        geo = geometric_approach(self.Lengths, self.secondary_dest_point)
        v1 = rotation_u([self.Lengths[0], 0, 0], [0, -1, 0], geo[1])
        v1 = rotation_u(v1, [0, 0, 1], geo[0])
        v2 = subtract(self.secondary_dest_point, v1)
        last_angle = 180 - math.degrees(angle(vector, v2))
        first_pair_normal = cross(v2, vector)
        second_pair_normal = cross(v1, v2)
        add_axis = math.degrees(angle(first_pair_normal, second_pair_normal))
        self.joint_angles = [geo[0], geo[1], geo[2], last_angle, add_axis]
        v1 = scalar_of_vector(make_unit(v1), self.Lengths[0])
        v2 = scalar_of_vector(make_unit(v2), self.Lengths[1])
        v3 = scalar_of_vector(vector, self.Lengths[2])
        self.vectors = [v1, v2, v3]

    # NOTE: Returning format: "base_yaw,base_pitch,secondary_axis,gripper_pitch,gripper_rotation"
    def return_model(self):
        str_msg = ""
        for i in range(0, len(self.joint_angles))
            str_msg += str(self.joint_angles[i])
        return str_msg


# TODO: Add rospy publish integration
    def publish(node_name):
        pass

    def print_info(self):
        print "Destination Point: " + str(self.destination_point)
        print "Gripper Point: " + str(self.secondary_dest_point)
        print "Direction Vector: " + str(self.direction_vector)
        print "Segment Lengths: " + str(self.Lengths)
        print "Joint Angles[base_yaw, base_pitch, secondary_axis, gripper_pitch, gripper_rotation]: " + str(self.joint_angles)
        print "Segment Vectors: " + str(self.vectors)
