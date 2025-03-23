#------PROGRAM STATE MACHINE CODE-------
#---------------------------------------
#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)
#WRITTEN BY ANDREW KRITIKOS COPYRIGHT 2025

import cameraInter as cam
import gpioInter as gpio 
import sensorData
import time
#----------------------------------------------------------------USER-DEFINED VARAIBLES---------------------------------------------------------------
interStDelay = 1 #default time to delay by default between states
intrSlack = 0.20 #distance (meters) between the stop line and the intersection
#------------------------------------------------------------NON-USER-DEFINED VARAIBLES---------------------------------------------------------------

#----------STATE DELAY----------
endTime = None
#----------GLOBAL CONDITIONAL VARS----------
currentState = "state_reset"
nextState = None

cameraDirectDevEnabled = False #enables directional device detecting for camera 
cameraLineTrackingEnabled = False #enables line and forward path verification for camera
cameraCornerTrackingEnabled = False #enables stop line detection for camera
#----------TRANSITIONAL CONDITIONAL BOOLEANS----------

isStopLineDetected = False
isObstaclePresent = False
isLineVisible = False
isCollisionBumperEngaged = False
foundSharpTurn = False
isTurnComplete = False
isDelayOver = True
isInterTypeIdentified = False
isTrafficLightDetected = False
foundPathForward = False
foundLeftStopLine = False
foundRightStopLine = False
isLightDirectionDetermined = False
turnDirection = None #String direction for which way to turn

#----------INTERSTATE CONDITIONS----------
stateIsTransitioning = False
involvedInIntersection = False

#----------COMPOUND TRANSITION COMBINATIONS
trans_leaveReset = [True, "state_Reset"] #CONDITIONLESS, MEANT TO GO IMMEDIATELY
trans_returnTurn = [(involvedInIntersection) and (not isObstaclePresent) and (isLineVisible) and (not isCollisionBumperEngaged), "state_ExecuteTurn"] #path is taken incase turn is interrupted by obstacle
trans_checkEmergency = [(isCollisionBumperEngaged) and (True), "state_Emergency"] #1
trans_stopLineDetected = [(isStopLineDetected),"state_StopLine"] #2
trans_returnSafeStop = [(isObstaclePresent) or (not isLineVisible) or (isCollisionBumperEngaged), "state_Safety_Stop"] #4
trans_beginDriving = [(not isObstaclePresent) and (isLineVisible) and (not isCollisionBumperEngaged), "state_FollowingLine"] #5
trans_goToIdentifyFeatures = [(not isObstaclePresent),"state_StopLine"]
trans_foundStraightThrough = [(isInterTypeIdentified) and (not isObstaclePresent) and (foundPathForward) and (isDelayOver) and (not foundLeftStopLine) and (foundRightStopLine),"state_FollowingLine"] #7
trans_yieldLeftThenForeward = [(not isObstaclePresent) and (foundPathForward) and (isDelayOver), "state_FollowingLine"] #8
trans_intersectionFound = [(not isObstaclePresent) and (isDelayOver), "state_IdentifyIntersection"] #11
trans_foundLeft90Turn = [(isInterTypeIdentified) and (not foundPathForward) and (not foundLeftStopLine) and (foundRightStopLine),"state_ExecuteTurn"] #12
trans_foundDirectionalDevice = [(isInterTypeIdentified) and (isTrafficLightDetected),"state_DetermineLight"] #13
trans_foundYieldLeft = [(isInterTypeIdentified) and (foundLeftStopLine) and (not foundRightStopLine), "state_YieldtoLeft"] #14
trans_lightTypeDetermined = [(isLightDirectionDetermined),"state_ExecuteTurn"] 
trans_yieldLefttoTurnRight = [(not isObstaclePresent) and (not foundPathForward),"state_ExecuteTurn"] #16
trans_turnFinished = [(isTurnComplete) and (isDelayOver),"state_FollowingLine"] #17
trans_sharpTurnEncountered = [(foundSharpTurn),"state_ExecuteTurn"]

stateNames = {
    # FORMAT:
        # Transition Conidtions [0] <- List of conditions that are searched through to determine if the state should change
        # Requisite Sensor Groups [1] <- List of sensors or parts of camera code that are searched through so the state only has to refresh important data
        # Post-Transition TRUE Resets [2] <- List of variables that need to be reset to TRUE after state transition happens
        # Post-Transition FALSE Resets [3] <- List of variables that need to be reset to FALSE after state transition happens
    
    "state_Reset": (
        (trans_leaveReset, trans_returnSafeStop),
        ("cameraLines","ultrasonic","colorRGB"),
        (None), #TRUE
        (None)), #
    "state_Safety_Stop": (  #regular car stop, activates for obstacles or intermediate
        (trans_checkEmergency,trans_beginDriving, trans_returnTurn),
        ("cameraLines","bumper","ultrasonic"),
        (None),
        (None)), 
    "state_FollowingLine": (
        (trans_checkEmergency,trans_returnSafeStop,trans_stopLineDetected),
        ("cameraLines","cameraCorners","bumper","ultrasonic","colorRGB"),
        (None),
        (None)),
    "state_StopLine": (
        (trans_checkEmergency,trans_intersectionFound,trans_returnSafeStop),
        ("cameraLines","cameraCorners","bumper","ultrasonic"),
        (None),
        (None)), 
    "state_IdentifyIntersection": (
        (trans_checkEmergency,trans_foundLeft90Turn,trans_foundDirectionalDevice,trans_foundYieldLeft,trans_foundStraightThrough, trans_returnSafeStop),
        ("cameraLines", "cameraCorners","bumper","cameraColors","ultrasonic"),
        (None),
        (None)),
    "state_YieldtoLeft": (
        (trans_checkEmergency,trans_yieldLeftThenForeward,trans_yieldLefttoTurnRight,trans_returnSafeStop),
        ("cameraLines","cameraCorners","bumper","ultrasonic"),
        (None),
        (None)),
    "state_DetermineLight": (
        (trans_checkEmergency,trans_lightTypeDetermined,trans_returnSafeStop),
        ("cameraLines","cameraCorners","bumper","cameraColors","ultrasonic"),
        (None),
        (None)),
    "state_ExecuteTurn": (
        (trans_checkEmergency,trans_turnFinished,trans_returnSafeStop),
        ("cameraLines","cameraCorners","bumper","cameraColors","ultrasonic"),
        (None),
        (None)),
    "state_Emergency": (
        (None),
        (None),
        (None))
}

#----------SECONDARY CONTROL-FLOW FUNCTIONS----------
def initalize():
    gpio.fntRed.off()
    gpio.fntGreen.on()
    gpio.resetGimbal()

def newDelay(customTime):
    global endTime,isDelayOver
    isDelayOver = False

    startTime = time.monotonic()
    if customTime is not None:
        endTime = time.monotonic() + customTime
    else:
        endTime = time.monotonic() + interStDelay

def checkDelayOver():
    global isDelayOver,endTime
    currentTime = time.monotonic()

    if (currentTime >= endTime):
        isDelayOver = True
        endTime = None
    else:
        isDelayOver = False
        
def resetStateVars():
    #reset all variables in current state list to respective TRUE or FALSE

    for item in stateNames[currentState][2]:
        if item != None:
            item = True
    for item in stateNames[currentState][3]:
        if item != None:
            item = False

    #NEEDS FIXING, INCOMPLETE
    sensorData.resetTruthData("obstacles","")

def fetchSensorData():

    global isStopLineDetected, isObstaclePresent, isLineVisible, isCollisionBumperEngaged, foundSharpTurn

    for i in stateNames[currentState][1]:
        match currentState[1]:
            case "bumper":
                gpio.pollBumpers()
                sensorData.interpretBumpers()
                print("Interpreting Bumper Sensors \n")
            case "ultrasonic":
                if (currentState == "state_IdentifyIntersection" or "state_YieldtoLeft"):
                    gpio.lookBothWays()
                else:
                    gpio.pollUltrasonic()
                sensorData.interpretSonicSensor()
            case "colorRGB":
                gpio.pollColorSens()
                sensorData.interpretColorSensors()
            case None:
                pass
            case _:
                raise Exception("INVALID SENSOR WAS ACCESSED")
    
    #assigns sensor data to dataFile

    isStopLineDetected = sensorData.stopLineDetected
    isObstaclePresent = sensorData.obstaclePresent
    isLineVisible = sensorData.lineIsVisible
    isCollisionBumperEngaged = sensorData.bumperEngaged
    foundSharpTurn = sensorData.sharpTurnDetected

    #REMAINING ASSIGNMENT VALUES:

    #isInterTypeIdentified = False
    #isTrafficLightDetected = False
    #foundPathForward = False
    #foundLeftStopLine = False #assigned by 
    #foundRightStopLine = False
    #isLightDirectionDetermined = False
    #turnDirection = None #String direction for which way to turn

def checkForChangeStates():
    checkDelayOver()

    for i in stateNames[currentState][0]:
        if (i[0] == True):
            nextState = i[1]

    if (nextState is not None):
        resetStateVars() #resets the required values for the state that was last exited
        currentState = nextState
        nextState = None #resets back to unchanged
        stateIsTransitioning = True        

#----------STATE FUNCTIONS----------

def state_Reset():
    initalize() #sets up sensors, lights, etc.
    resetStateVars() #reset all functions
    sensorData.resetTruthData("obstacles","bumpers","lineDetection")

    while(not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()

def state_Safety_Stop():
    gpio.emergencyStop()

def state_FollowingLine():
    cameraLineTrackingEnabled = True
    gpio.resetGimbal()
    while(not stateIsTransitioning):
        gpio.updateMotion('drive',)
        fetchSensorData()
        checkForChangeStates()

def state_IdentifyIntersection():
    involvedInIntersection = True
    while (not stateIsTransitioning):
        
        fetchSensorData()
        checkForChangeStates()

def state_StopLine():
    global involvedInIntersection

    involvedInIntersection = True
    gpio.halt()
    while (not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()

def state_YieldtoLeft():
    global involvedInIntersection

    involvedInIntersection = True
    while (not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()

def state_DetermineLight():
    global involvedInIntersection

    involvedInIntersection = True
    while (not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()

def state_ExecuteTurn():
    global involvedInIntersection

    involvedInIntersection = True
    gpio.halt()
    gpio.moveIncremental('f',0.2)
    while(sensorData.sensorData["colorSens"][]): #move up to intersection phase
        fetchSensorData()
        gpio.moveIncremental('f',0.01)
    while(): #rotation phase
        fetchSensorData()
        if(turnDirection == "right"):
            gpio.moveIncremental('cnclk',0.01)
        else:
            gpio.moveIncremental('cnclk',0.01)
    while (not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()
    #reset camera gimbal
    involvedInIntersection = False

def state_Emergency():
    gpio.fntRed.on() #red LED on
    gpio.fntGreen.off() #green LED off
    gpio.emergencyStop() #emergency stop motors
    print("EMERGENCY STOP BUMPER WAS ACTIVATED, CONTROL TERMINATED!")
    while(1): #forces FSM to stop in infinite loop
        pass

#----------PRIMARY CONTROL-FLOW FUNCTIONS----------

def ctrlLoop():
    global stateIsTransitioning

    resetStateVars()
    initalize()
    while(1): #Primary loop
        match currentState:
            case "state_Reset":
                state_Reset()
            case "state_Safety_Stop":
                state_Safety_Stop()
            case "state_FollowingLine":
                state_FollowingLine()
            case "state_StopLine":
                state_StopLine()
            case "state_IdentifyIntersection":
                state_IdentifyIntersection()
            case "state_YieldtoLeft":
                state_YieldtoLeft()
            case "state_DetermineLight":
                state_DetermineLight()
            case "state_ExecuteTurn":
                state_ExecuteTurn()
            case "state_Emergency":
                state_Emergency()
            case _:
                raise Exception("INVALID STATE ID HAS BEEN ACCESSED")
    stateIsTransitioning = False
    print ("State has now been changed to {}".format(currentState))
                
    print ("FSM HAS LEFT THE CONTROL LOOP")

#----------PROGRAM INITIALIZATION----------

ctrlLoop() #run FSM and rest of program