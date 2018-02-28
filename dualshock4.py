import hid

class PS4Controller():
    """DOC"""
    def __init__(self, vendor, product):
        self.h = hid.device()
        self.h.open(0x046d, 0xc215)
        self.h.set_nonblocking(1)
        self.raw_data = []
        self.angles_raw = [0., 0., 0.]
        self.gyro_raw = [0., 0., 0.]
        self.accel_raw = [0., 0., 0.]
        self.axis_raw = [0., 0., 0., 0., 0., 0.]
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def update(self):
        self.raw_data = self.h.read(72)

        # self.angles_raw[0] = self.raw_data[22] # Yaw
        # self.angles_raw[1] = self.raw_data[24] # Pitch
        # self.angles_raw[2] = self.raw_data[20] # Roll
        #
        # self.gyro_raw[0] = self.raw_data[21] # Yaw
        # self.gyro_raw[1] = self.raw_data[23] # Pitch
        # self.gyro_raw[2] = self.raw_data[19] # Roll
        #
        # self.accel_raw[0] = self.raw_data[13] # x
        # self.accel_raw[1] = self.raw_data[15] # y
        # self.accel_raw[2] = self.raw_data[17] # z
        #
        # self.axis_raw[0] = self.raw_data[1] # Left-x
        # self.axis_raw[1] = self.raw_data[2] # Left-y
        # self.axis_raw[2] = self.raw_data[3] # right-x
        # self.axis_raw[3] = self.raw_data[4] # right-y
        # self.axis_raw[4] = self.raw_data[8] # left-trigger
        # self.axis_raw[5] = self.raw_data[9] # right-trigger

# t = PS4Controller(0x046d, 0xc215)
# while True:
#     t.update()
#     print t.raw_data
