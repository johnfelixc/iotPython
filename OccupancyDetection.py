import sys
import os
import time
#import statistics
import copy

from pirSensor import pirSensor
from echoSensor import echoSensor
from SensorUDPClient import SensorUDPClient

class OccupancyDetection():
    objectSensor = 0
    humanSensor = 0
    OccupancyStatus = False
    client = SensorUDPClient("localhost", 10000)
    
    def __init__(self):
        self.objectSensor = echoSensor()
        self.humanSensor = pirSensor()

    def initializeSensors(self):
        self.objectSensor.initSensors()
        self.humanSensor.initSensors()

    def getMergedSensorData(self):
        mergedSensorData = {}
        objectData = self.objectSensor.getSensorData()
        humanData = self.humanSensor.getSensorData()

        objectSize = len(objectData)
        humanSize = len(humanData)
        
        for elem in objectData:
            timeValue = int(elem[0])
            mergedSensorData[timeValue] = {}
            mergedSensorData[timeValue].update({0:elem[1]})

        for elem in humanData:
            timeValue = int(elem[0])
            if not (timeValue in mergedSensorData):
                mergedSensorData[timeValue] = {}
            mergedSensorData[timeValue].update({1:elem[1]})
  
        return mergedSensorData

    def processSample(self, sensorSampleData):
        nextSampleTime = min(sensorSampleData.keys())
        
        currSampledPIRValue = False
        currSampledDistanceParams = {}
        keyDistanceValues = []
        distanceValues = []
        
        for timeVal, record in sorted(sensorSampleData.items()):
            if len(record.keys()) < 2:
                continue
            currSampledPIRValue = currSampledPIRValue or record[1]
            if record[0]:
                distanceValues.append(record[0])
                if record[1] :
                    keyDistanceValues.append(record[0])
            
            if timeVal > nextSampleTime :
                distanceStat = []
                row = []
                distanceStat.append(max(distanceValues))
                distanceStat.append(min(distanceValues))

                row.append(currSampledPIRValue)
                row.append(copy.deepcopy(distanceStat))
                row.append(copy.deepcopy(keyDistanceValues))
                currSampledDistanceParams[timeVal] = row

                distanceStat.clear()
                distanceValues.clear()
                keyDistanceValues.clear()
                currSampledPIRValue = False
                nextSampleTime = timeVal + 10
                                    
        return currSampledDistanceParams

    def checkOccupancy(self, sensorData):

        for timeVal, sensorEvent in sorted(sensorData.items(), reverse=True):
            #Checking PIR Sensor Value
            if not sensorEvent[0]:
                continue

            #Check time
            timeDiff = time.time() - timeVal
            if timeDiff > 30:
                break

            #Check number of key Distances
            if len(sensorEvent[2]) < 2:
                continue

            #Distance Check
            if min(sensorEvent[1]) > 20:
                continue

            return True
            
        return False

    def checkAvailability(self, sensorData):
        sampleCount = 0
        for timeVal, sensorEvent in sorted(sensorData.items(), reverse=True):
            #Checking PIR Sensor Value
            if 1 in sensorEvent:
                if sensorEvent[1]:
                    return False
                else:
                    sampleCount = sampleCount + 1
                    if sampleCount > 60:
                        return True
        return False

    def startDetection(self):
        self.objectSensor.startReading()
        self.humanSensor.startReading()
        
    
        endTime = time.time() + 120

        while True:
            time.sleep(1)
            sensorSampleData = self.getMergedSensorData()
            processedSensorData = self.processSample(sensorSampleData)

            if self.OccupancyStatus:
                if self.checkAvailability(sensorSampleData):
                    self.OccupancyStatus = False
                    print("\t========Available==========\t")
            else:
                if self.checkOccupancy(processedSensorData):
                    self.OccupancyStatus = True
                    print("\t=========Occupied=========\t")

            maxKey = max(sensorSampleData.keys())
            while len(sensorSampleData[maxKey]) < 2:
                maxKey = maxKey - 1
            self.client.sendSensorData(self.OccupancyStatus, sensorSampleData[maxKey][1], sensorSampleData[maxKey][0])
                                                             
    
        self.objectSensor.stopReading()
        self.humanSensor.stopReading()

############################
## Main Code Block
############################

detector = OccupancyDetection()

detector.initializeSensors()
detector.startDetection()

