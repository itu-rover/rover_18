import socket
import time
loop_rate = 1000

host = "0.0.0.0"
port = 9090

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.settimeout(1000)
client.connect((host, port))


while True:
    client.send("TEST\n")
    time.sleep(1.0 / loop_rate)
client.close()
