import serial
import sys
import RPi.GPIO as GPIO      
import os, time
from decimal import *
from struct import *
 
# Find a character in a string
def find(binString, ch):
    for i, ltr in enumerate(binString):
        if ltr == ch:
            yield i
 
#Enable Serial communication
port = serial.Serial("/dev/serial0", 9600, timeout=1)
time.sleep(4)
frame = bytearray()
bytesRead = b""
print("Baudrate : " + str(port.baudrate))
try:
    
    while True:
        try:
            bytesRead = port.readline()  # Read the data from the port
        except serial.SerialException:
            time.sleep(2)
            port.close()
            port = serial.Serial("/dev/serial0", 9600, timeout=1)
            continue
        # print(bytesRead) 
     
        if len(bytesRead)>5:  # len(rcv)>5:
            # Data Extraction
            L1=list(find(bytesRead, 68))
            L2=list(find(bytesRead, 84))
            Dist=bytesRead[(L1[0]+1):(L2[0])]
            # print(Dist)
            # fDist=unpack("f", pack("I", bits2int(Dist)))
            print(Dist.decode("utf-8") + " cm")
except serial.SerialException:
    print("Caught Serial Exception")
    # sys.exit(1)
finally:
    GPIO.cleanup()
    print("Closing Port")
    port.close()
