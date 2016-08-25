import socket
import sys
import struct

class SensorUDPClient():

        hostname = "localhost"
        port = 10000

        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = (hostname, port)

        def __init__(self, hostname, port):
                self.hostname = hostname
                self.port = port
                server_address = (hostname, port)
                return

        def sendSensorData(self, Occupancy, PIR, Distance):
                binData = struct.pack("<??f", Occupancy, PIR, Distance)
                sent = self.sock.sendto(binData, self.server_address)
                return

        def closeClient():
                sock.close()
                return
