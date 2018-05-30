from server.socket_server import BasicChatServer
import thread
import serial
import time

loop_rate = 25

def establish_serial_connection(portname='/dev/cu.usbserial-A9S7BXXD', baud=115200, _timeout=0.3):
    ser = serial.Serial(
        # 0,
    	port=portname,
    	baudrate=baud,
        timeout=_timeout
    )
    try:
        ser.close()
        ser.open()
    except serial.SerialException:
        print "[ ERROR ] Serial port opening failed. No such serial port available, aborting!"
    return ser

def serial_write(serial_port, data):
    data = data.strip()
    msg = data + "\r\n"
    # msg = "HELLOWORLD!\r\n"
    # if self.ser.isOpen():
    #self.ser.write(string)
    try:
        # serial_port.write(msg.encode())
        # serial_port.flush()
        print "[ SERIAL ]", msg, serial_port
    except serial.SerialException:
        print "[ ERROR ] Serial writing failed, aborting!"

def callback(client, data, port):
    serial_write(port, data)
    time.sleep(1.0 / loop_rate)

def main():
    server = BasicChatServer(callback)
    # port = establish_serial_connection()
    port = None
    thread.start_new_thread(server.run, (port,))
    while True:
        time.sleep(10)

if __name__ == "__main__":
    main()
