from socket import *

HOST = '172.31.1.147'
PORT = 30003
ADDR = (HOST,PORT)

UdpSocket = socket(AF_INET, SOCK_DGRAM)

points = []
points.append("530 150 400 0 0 -1 0")
points.append("730 150 400 0 0 -1 0")
points.append("730 -150 400 0 0 -1 0")
points.append("530 -150 400 0 0 -1 0")

pocket = str.encode(";".join(points))

UdpSocket.sendto(pocket, ADDR)
print("Send", len(pocket))