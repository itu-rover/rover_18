import vectorial_calculations as v_calc

class Navigation(object):
    """The navigation class for navigating the robotic arm"""
    def __init__(self, initial_position, initial_vector):
        self.position = initial_position
        self.vector = initial_vector

    def zoom(self, zooming_vel, update=True):
        # Ensure the approaching vector is unit size.
        self.vector = v_calc.make_unit(self.vector)

        # Multiply the approaching vector with a scalar
        vector_temp = v_calc.scalar_of_vector(self.vector, zooming_vel)

        # Add the vector to the position to get moved in the approaching vector's direction
        self.position = v_calc.sum_vector(self.position, vector_temp)


    """
        right_left_vel > 0 : moving right
        rigth_left_vel < 0 : moving left
        considering r_l_directional_vector points to the right
    """
    def rigth_left(self, r_l_directional_vector, right_left_vel):
        # Ensure the approaching vector is unit size.
        r_l_directional_vector = v_calc.make_unit(r_l_directional_vector)

        # Multiply the approaching vector with a scalar
        vector_temp = v_calc.scalar_of_vector(r_l_directional_vector, right_left_vel)

        # Add the vector to the position to get moved in the approaching vector's direction
        self.position = v_calc.sum_vector(self.position, vector_temp)


    """
        up_down_vel > 0 : moving up
        up_down_vel < 0 : moving down
        considering up_down_directional_vector points to the up
    """
    def up_down(self, up_down_directional_vector, up_down_vel):
        # Ensure the approaching vector is unit size.
        up_down_directional_vector = v_calc.make_unit(up_down_directional_vector)

        # Multiply the approaching vector with a scalar
        vector_temp = v_calc.scalar_of_vector(up_down_directional_vector, up_down_vel)

        # Add the vector to the position to get moved in the approaching vector's direction
        self.position = v_calc.sum_vector(self.position, vector_temp)

    def rotate_yaw(self, up_down_directional_vector, rotate_degrees):
        v_temp = self.vector
        v_temp = v_calc.rotation_u(v_temp, up_down_directional_vector, -rotate_degrees)
        self.vector = v_temp

    def rotate_pitch(self, r_l_directional_vector, rotate_degrees):
        v_temp = self.vector
        v_temp = v_calc.rotation_u(v_temp, r_l_directional_vector, rotate_degrees)
        self.vector = v_temp
