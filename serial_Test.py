import serial
from time import sleep
ser = None

def establish_serial_connection():
    global ser
    ser = serial.Serial(
        # 0,
    	port='/dev/cu.usbserial-A9TMWTD3',
    	baudrate=115200,
        timeout=0.3
    )
    ser.close()
    ser.open()
j = 0

def serial_write():
    global ser, j
    # string = self.return_model_for_low_level()
    string = "SD" + str(j) + "F\r\n"
    print string.strip()
    j += 1
    # if self.ser.isOpen():
    #self.ser.write(string)
    ser.write(string.encode())
    ser.flush()


establish_serial_connection()
while True:
    serial_write()
    sleep(0.3)
