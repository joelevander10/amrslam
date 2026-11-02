import socket
import time
ip = "192.168.3.200"
port = 2022

conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
conn.connect((ip, port))

while True:
    response = conn.recv(1024)
    print(response.hex().upper())
    print()
    time.sleep(0.5)