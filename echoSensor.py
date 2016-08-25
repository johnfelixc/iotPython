import serial
import sys   
import os, time
import copy
from threading import Thread
from collections import deque

SERIAL_PATH = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 9600
QUEUE_SIZE = 3600

class echoSensor():
    sensorPort = 0
    distanceQueue = deque(maxlen=QUEUE_SIZE)
    temperatureQueue = deque(maxlen=QUEUE_SIZE)
    threadReadStatus = False
# Constructor
    def __init__(self):
        return

# Public Functions

    def initSensors(self):
        self.sensorPort = self.connectPort(SERIAL_PATH, SERIAL_BAUDRATE)
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
        return copy.deepcopy(self.distanceQueue)

    def deinitSensors(self):
        if self.sensorPort.isOpen:
            print("Closing Echo Sensor Port")
            if self.sensorPort.isOpen:
                #print(self.sensorPort)
                self.sensorPort.close()
    
    
# Private Functions

    def connectPort(self, path, baudrate):
        print("Connecting : " + path)
        self.sensorPort = serial.Serial(path, SERIAL_BAUDRATE, timeout=1)
        print("Baudrate : " + str(self.sensorPort.baudrate))
        time.sleep(1)
        return self.sensorPort

    def reconnectPort(self, path, baudrate):
        sensorPort.close()
        sensorPort = serial.Serial(path, 9600, timeout=1)
        time.sleep(1)
        return True
    
    def readingThread(self):
        byte = b""
        Distance = 0
        DistanceUnit = 2
        Temperature = 0
        TemperatureUnit = 2
        DistanceSensor = False
        TemperatureSensor = False
        print("Distance Sensor Thread Started")
        #print(self.sensorPort)
        nextSampleTime = time.time()
        while self.sensorPort.isOpen:
            try:
                byte = self.sensorPort.read()  # Read the data from the port
                #print("Read: " + byte.decode())
                byteInt = ord(byte)
                #print("Read: " + byte.decode("utf-8") + " | " + str(byteInt))
                if byteInt is 10 or byteInt is 13:
                    currTime = time.time()
                    if currTime > nextSampleTime:
                        nextSampleTime = currTime + 1
                        if Distance is not 0:
                            self.distanceQueue.append([currTime, Distance])
                            #print(Distance)
                        #print(self.distanceQueue)
                        self.temperatureQueue.append([currTime, Temperature])
                        #print(self.temperatureQueue)
                        continue
                    TemperatureSensor = False
                    DistanceSensor = False
                elif byteInt > 47 and byteInt < 58:
                    intConv = byteInt - 48
                    if DistanceSensor:
                        Distance += pow(10, DistanceUnit) * intConv
                        DistanceUnit = DistanceUnit - 1
                    elif TemperatureSensor:
                        Temperature += pow(10, TemperatureUnit) * intConv
                        TemperatureUnit = TemperatureUnit - 1
                    else:
                        print("Sensor not selected")
                elif byteInt is 68:
                    #print("Setting Distance Sensor")
                    DistanceSensor = True
                    TemperatureSensor = False
                    Distance = 0
                    DistanceUnit = 2
                elif byteInt is 84:
                    #print("Setting Temperature Sensor")
                    TemperatureSensor = True
                    DistanceSensor = False
                    Temperature = 0
                    TemperatureUnit = 2
                elif byteInt is 46:
                    #print("Setting Decimal unit")
                    if DistanceSensor:
                        DistanceUnit = -1
                    elif TemperatureSensor:
                        TemperatureUnit = -1
                    else:
                        print("Sensor not selected")
                else:
                    print("Unknown byte read: " + byte.decode())
            except (serial.SerialException, BlockingIOError, OSError):
                print("Caught Exception")
                self.sensorPort = reConnect(port, SERIAL_PATH, SERIAL_BAUDRATE)
                #break
                continue
            if not self.threadReadStatus:
                print(self.threadReadStatus)
                self.deinitSensors()
                break
        print("Distance Sensor Thread exiting")
 
###########################
#Test code block ##########
###########################

##distSensor = echoSensor()
##
##try:   
##    distSensor.initSensors()
##    distSensor.startReading()
##    time.sleep(10)
##    distSensor.stopReading()
##
##except serial.SerialException as err:
##    print("Caught Serial Exception")
##    print(err)
##    # sys.exit(1)
##finally:
##    print("Finally")
##    #GPIO.cleanup()
##    #distSensor.deinitSensors()
##    #port.close()
