import RPi.GPIO as GPIO
import serial
import os
import time
import collections
from decimal import *


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.IN)         #Read output from PIR motion sensor
prevStatus = 0
currStatus = 0
lastOccupiedTime = time.time()
lastOccupiedTimeQueue = collections.deque(maxlen=4)

#Enable Serial communication
port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.1)

# Find a character in a string
def find(str, ch):
    for i, ltr in enumerate(str):
        if ltr == ch:
            yield i
            
# Compute Time difference is a list
def computeTimeDiff(listTime):
    diffTime = 0
    sizeQ = len(listTime)
    
    if sizeQ < 2:
        return diffTime
    
    for i, x in enumerate(listTime):
        if i > 0:
            diffTime = diffTime + (listTime[i] - listTime[i-1])
    print(diffTime)
    return diffTime / (sizeQ - 1)

while True:
    currStatus = GPIO.input(11)
    rcv = port.read(10)  # Read the data from the port
    
    if currStatus == 0 and prevStatus == 1:     #When output from motion sensor is LOW        
        timeDur = time.time() - lastOccupiedTime
        if timeDur > 60:
            print("Available")
            prevStatus = currStatus
            lastOccupiedTimeQueue.clear()
        time.sleep(1)
        
    elif currStatus == 1:                       #When output from motion sensor is HIGH
        lastOccupiedTimeQueue.append(time.time())
        timeAvgDur = computeTimeDiff(list(lastOccupiedTimeQueue))
        if prevStatus == 0 and timeAvgDur < 10  and len(lastOccupiedTimeQueue) > 2:
            print("Occupied")
            lastOccupiedTime = time.time()
            prevStatus = currStatus
        time.sleep(2)
        lastOccupiedTime = time.time()

    if len(rcv)>5:
        # Data Extraction
        L1=list(find(rcv, "D"))
        L2=list(find(rcv, "T"))
        Dist=rcv[(L1[0]+1):(L2[0])]
        Dist=Decimal(Dist)
        print(rcv)
        print(str(Dist) + " cm")
    
GPIO.cleanup()
