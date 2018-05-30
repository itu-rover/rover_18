from p2p import generate_curve, generate_path
from vectorial_calculations import subtract, length, make_unit, scalar_of_vector, sum_vector
import time

grippers = {
    "GRIP1": [[100, 0, 30], [1, 0, 0]],
    "GRIP2": [[100, 0, 30], [1, 0, 0]],
    "GRIP3": [[100, 0, 30], [1, 0, 0]]
}


class AdaptiveGripperClient():
    def __init__(self, arm, mode, callback, safety_const=15):
        if not mode in grippers.keys():
            print "[ ERROR ] Unsupported gripper type. Aborting."
            return
        target = grippers[mode]
        self.target_destination = target[0]
        self.target_direction = target[1]
        self.arm = arm
        self.safety_const = safety_const
        self.points = [0, 0, 0]
        self.callback = callback


    def move(self):
        safe_approach_point = sum_vector(self.target_destination, scalar_of_vector(make_unit(self.target_direction), -self.safety_const))
        self.points, self.dirs, avg_step = generate_curve(self.arm.current_position, self.arm.current_direction, safe_approach_point, self.target_destination, 20, 0.1, 0.1)

        _points = generate_path(safe_approach_point, self.target_destination, avg_step)
        self.dirs.pop(len(self.dirs) - 1)
        for i in _points:
            self.points.append(i)
            self.dirs.append(self.dirs[len(self.dirs) - 1])

        if len(self.dirs) != len(self.points):
            print "[ ERROR ] Direction and position vectors does not have equal size. Aborting."
            return False

        for i in range(len(self.dirs)):
            (state, _points) = self.arm.update_destination_point(self.points[i], self.dirs[i])
            if not state:
                print ":"
                self.callback(_points)
            else:
                self.callback()
            time.sleep(0.1)
            # self.arm.serial_write()
