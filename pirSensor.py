import RPi.GPIO as GPIO
import sys   
import os, time
import copy
from threading import Thread
from collections import deque


SENSOR_GPIO_PORT = 11
QUEUE_SIZE = 3600

class pirSensor():
    occupancyQueue = deque(maxlen=QUEUE_SIZE)
    threadReadStatus = False
# Constructor
    def __init__(self):
        return

# Public Functions

    def initSensors(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SENSOR_GPIO_PORT, GPIO.IN) 
        return

    def startReading(self):
        threadID = Thread(target=self.readingThread)
        threadID.daemon = True
        self.threadReadStatus = True
        threadID.start()       
        return True

    def stopReading(self):
        self.threadReadStatus = False
        return True

    def getSensorData(self):
        return copy.deepcopy(self.occupancyQueue)

    def getCurrentSensorValue():
        currStatus = GPIO.input(SENSOR_GPIO_PORT)
        if currStatus is 0:
            currStatus = False
        else:
            currStatus = True
        return currStatus

    def deinitSensors(self):
        print("Cleaning GPIO")
        GPIO.cleanup()


   
    
# Private Functions

    def readingThread(self):
        print("PIR Sensor Read Thread Started")
        nextSampleTime = time.time()
        currSampledValue = False
        while self.threadReadStatus:
            currTime = time.time()
            
            try:
                if nextSampleTime > currTime and currSampledValue:
                    continue
                currStatus = GPIO.input(SENSOR_GPIO_PORT)
                if currStatus is 0:
                    currStatus = False
                else:
                    currStatus = True

                currSampledValue = currSampledValue or currStatus

                if currTime > nextSampleTime:
                    self.occupancyQueue.append([currTime, currStatus])
                    nextSampleTime = currTime + 1
                    currSampledValue = False

                time.sleep(0.1)

                #print(self.occupancyQueue)                
            except (BlockingIOError, OSError):
                print("Caught Exception")
                #print(err)
                continue
            
        print("PIR Sensor Read Thread exiting")
        self.deinitSensors()
 
###########################
#Test code block ##########
###########################

##distSensor = pirSensor()
##distSensor.initSensors()
##distSensor.startReading()
##time.sleep(60)
##distSensor.stopReading()
