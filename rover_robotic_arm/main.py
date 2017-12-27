from robotic_arm import RoverArm

test = RoverArm([50, 40, 15])
test.update_destination_point([40,0,40], [1,0,0])
test.print_info()
