from robotic_arm import RoverArm

test = RoverArm([50, 40, 15])

f = open("s.txt", 'w')
f.write("0.00\t0.00\t0.00\n")
number = 5
for i in range(-50, 50):
    for j in range(10, 50):
        for k in range(70, 160):
            for l in range(-10, 10):
                for m in range(-5, 5):
                    if i % number == 0 and j % number == 0 and k % number == 0 and l % number == 0 and m % number == 0:
                        f.write(str(test.foward_model([i, j, k, l, m])[2]).split('[')[1].split(']')[0].replace(",", "\t") + "\n")
    print str((i - 10) * 5 / 2) + " percent"
f.close()
