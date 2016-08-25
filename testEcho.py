import serial
import sys
import RPi.GPIO as GPIO      
import os, time
from decimal import *
from struct import *

SERIAL_PATH = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 9600
 
# Find a character in a string
def find(binString, ch):
    for i, ltr in enumerate(binString):
        if ltr == ch:
            yield i

def connect(path, baudrate):
    print("Connecting : " + path)
    port = serial.Serial(path, 9600, timeout=1)
    print("Baudrate : " + str(port.baudrate))
    time.sleep(1)
    return port

def reConnect(port, path, baudrate):
    port.close()
    port = serial.Serial(path, 9600, timeout=1)
    time.sleep(1)
    return port


#Enable Serial communication
port = connect(SERIAL_PATH, SERIAL_BAUDRATE)

bytesRead = b""
byte = b""
Distance = 0
DistanceUnit = 2
Temperature = 0
TemperatureUnit = 2
DistanceSensor = False
TemperatureSensor = False
try:
    
    while port.isOpen:
        try:
            byte = port.read()  # Read the data from the port
            #print("Read: " + byte.decode("utf-8"))
            byteInt = ord(byte)
            #print("Read: " + byte.decode("utf-8") + " | " + str(byteInt))
            if byteInt is 10 or byteInt is 13:
                print("Distance: " + str(Distance) + " cm")
                print("Temperature: " + str(Temperature) + " C")
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
            port = reConnect(port, SERIAL_PATH, SERIAL_BAUDRATE)
            continue
        # print(bytesRead) 
except serial.SerialException as err:
    print("Caught Serial Exception")
    print(err)
    # sys.exit(1)
finally:
    #GPIO.cleanup()
    print("Closing Port")
    port.close()
