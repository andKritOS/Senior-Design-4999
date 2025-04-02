#---------FUNCTIONS FOR FETCHING AND INTERPRETING SENSOR DATA---------
#---------------------------------------------------------------------
#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)
#WRITTEN BY ANDREW KRITIKOS COPYRIGHT 2025

#----------EXTERNAL SENSOR DATA---------------

sensorData = {
    "ultraSonic":[0,0,0], #ultrasonic latest readings [Left,Center,Right]
    "colorSensor":[[0,0,0], [0,0,0], [0,0,0]], # [L[RGB] (0), C[RGB] (1), R[RGB] (2)]
    #"bumpers": [0,0,0] #if this ever becomes 1, something went wrong and the robot will stop
}

#----------THRESHOLD VALUES----------
_BlueThreshMin = (86,105,140)
_BlueThreshMax = (0,0,255)

_ultSonicThreshObstacle = 0.3

#----------TRUTH DATA----------

#list out all camera frame types as individual variables

#-----Color Sensor Exclusive Truths
_blueOnLeft = False
_blueOnRight = False
_blueOnCenter = False

#-----Bumper Sensor Exclusive Truths
bumperEngaged = False

#-----Camera Exclusive Truths
leftLightDetected = False
yellowLightDetected = False
rightLightDetected = False

cameraLineTrackingEnabled = False
cameraCornerTrackingEnabled = False
cameraColorTrackingEnabled = False

#-----Ultrasonic Exclusive Truths
obstacleOnLeft = False
obstacleOnRight = False
obstacleAhead = False
obstaclePresent = False

#-----Color Sensor Exclusive Truths
turnLeftColorSensor = False
turnRightColorSensor = False

#-----Compound and State Machine Dependent Truths
lineIsVisible = False
stopLineDetected = False
sharpTurnDetected = False

foundLeftStopLine = False
foundRightStopLine = False
foundPathForward = False
trafficLightDetected = False

lightDirectionDetermined = False
intersectionTypeIdentified = False
turnDirection = None #string type

#----------FILE-VISIBLE WRITE FUNCTIONS----------

#def writeBumperSensorData(sensorString: str, data):
#    match sensorString:
#        case "bumperLeft":
#            sensorData.sensorData["bumpers"][0] = round(data)
#        case "bumperCenter":
#            sensorData.sensorData["bumpers"][1] = round(data)
#        case "bumperRight":
#            sensorData.sensorData["bumpers"][2] = round(data)

def writeColorSensorData(sensorString: str, data):
    match sensorString:
        case "colorLeft":
            sensorData.sensorData["colorSensor"][0] = round(data)
        case "colorCenter":
            sensorData.sensorData["colorSensor"][1] = round(data)
        case "colorRight":
            sensorData.sensorData["colorSensor"][2] = round(data)

def writeSonicSensorData(sensorString: str, data):
    match sensorString:
        case "ultraSonicLeft":
            sensorData.sensorData["ultraSonic"][0] = round(data * 100)
        case "ultraSonicCenter":
            sensorData.sensorData["ultraSonic"][1] = round(data * 100)
        case "ultraSonicRight":
            sensorData.sensorData["ultraSonic"][2] = round(data * 100)

#----------FILE-VISIBLE INTERPRET FUNCTIONS----------

#-----Color Sensors
def interpretColorSensors(sensorNum):

    global blueOnLeft,blueOnCenter,blueOnRight,lineIsVisible,stopLineDetected,turnLeftColorSensor,turnRightColorSensor,sharpTurnDetected

    match sensorNum:
        case 0:
            if all(_BlueThreshMin[i] <= sensorData["colorSensor"][0][i] <= _BlueThreshMax[i] for i in range(0,2)):
                blueOnLeft = True
        case 1:
            if all(_BlueThreshMin[i] <= sensorData["colorSensor"][1][i] <= _BlueThreshMax[i] for i in range(0,2)):
                blueOnCenter = True
        case 2:
            if all(_BlueThreshMin[i] <= sensorData["colorSensor"][2][i] <= _BlueThreshMax[i] for i in range(0,2)):
                blueOnRight = True
        case None:
            if all(_BlueThreshMin[i] <= sensorData["colorSensor"][0][i] <= _BlueThreshMax[i] for i in range(0,2)):
                blueOnLeft = True
            if all(_BlueThreshMin[i] <= sensorData["colorSensor"][1][i] <= _BlueThreshMax[i] for i in range(0,2)):
                blueOnCenter = True
            if all(_BlueThreshMin[i] <= sensorData["colorSensor"][2][i] <= _BlueThreshMax[i] for i in range(0,2)):
                blueOnRight = True

    lineIsVisible = _blueOnCenter #one is equivalent to another
    stopLineDetected = blueOnLeft and _blueOnCenter and _blueOnRight
    sharpTurnDetected = (not blueOnLeft and not blueOnCenter) or (not blueOnRight and not blueOnCenter)
    turnLeftColorSensor = blueOnLeft
    turnRightColorSensor = blueOnRight

#-----Bumper Sensors
#def interpretBumpers():
    
    #global bumperEngaged

    #if any(sensorData["bumpers"]):
        #bumperEngaged = True

#-----UltraSonic Sensors
def interpretSonicSensor(directionString: str):

    global obstacleOnLeft,obstacleAhead,obstacleOnRight, obstaclePresent

    obstacleAhead = False
    obstacleOnLeft = False
    obstacleOnRight = False

    match directionString:
        case "left":
            if (sensorData["ultraSonic"][0] <= _ultSonicThreshObstacle):
                obstacleOnLeft =  True
        case "center":
            if (sensorData["ultraSonic"][1] <= _ultSonicThreshObstacle):
                obstacleAhead =  True
        case "right":
            if (sensorData["ultraSonic"][2] <= _ultSonicThreshObstacle):
                obstacleOnRight =  True
        case None:

            if (sensorData["ultraSonic"][0] <= _ultSonicThreshObstacle):
                obstacleOnLeft =  True
            if (sensorData["ultraSonic"][1] <= _ultSonicThreshObstacle):
                obstacleAhead =  True
            if (sensorData["ultraSonic"][2] <= _ultSonicThreshObstacle):
                obstacleOnRight =  True

    obstaclePresent = obstacleOnLeft or obstacleAhead or obstacleOnRight

#----------MISC FUNCTIONS

def interpretCamera_Colors():
    global turnDirection
    if (leftLightDetected):
        turnDirection = "left"
    elif (rightLightDetected):
        turnDirection = "right"
    else:
        turnDirection = "none"

def interpretCamera_Corners():
    pass

#----------DATA-RESET FUNCTIONS----------
def resetTruthData(sensorString: str):

    #IF THERE IS EVER A PROBLEM WITH DATA WRITING OR ANY OTHER DETECTION, CHECK IT HERE

    global blueOnLeft,blueOnCenter,blueOnRight,obstacleOnLeft,obstacleAhead,obstacleOnRight,bumperEngaged,_blueOnLeft,_blueOnCenter,_blueOnRight
    global lineIsVisible,stopLineDetected,leftStopLineDetected,rightStopLineDetected, obstaclePresent, sharpTurnDetected

    for each in sensorString:
        match each:
            case "obstacles":
                obstacleOnLeft = False
                obstacleOnRight = False
                obstacleOnRight = False
                obstaclePresent = False
            case "bumpers":
                print("Error, why would you ever want to clear the bumper data? You crashed...\n")
                bumperEngaged = False
            case "lineDetection":
                blueOnLeft = False
                blueOnRight = False
                blueOnCenter = False
                lineIsVisible = False
                stopLineDetected = False
                sharpTurnDetected = False
            case _:
                print("ERROR, INVALID DATA RESET CALL\n")

def resetSensorData():
    global sensorData
    sensorData["ultraSonic"][0] = 0
    sensorData["ultraSonic"][1] = 0
    sensorData["ultraSonic"][2] = 0

    for i in range(0,2):
        for j in range(0,2):
            sensorData["colorSensor"][i][j] = 0

    sensorData["bumpers"][0] = 0
    sensorData["bumpers"][1] = 0
    sensorData["bumpers"][2] = 0
