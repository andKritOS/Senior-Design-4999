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
interStDelay = 100 #default time (ns) to delay by default between states
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
        ("cameraLines","ultrasonic","colorSensors"),
        (None), #TRUE
        (None)), #FALSE
    "state_Safety_Stop": (  #regular car stop, activates for obstacles or intermediate
        (trans_checkEmergency,trans_beginDriving, trans_returnTurn),
        ("cameraLines","bumper","ultrasonic"),
        (None),
        (None)), 
    "state_FollowingLine": (
        (trans_checkEmergency,trans_returnSafeStop,trans_stopLineDetected),
        ("cameraLines","cameraCorners","bumper","ultrasonic","colorSensors"),
        (None),
        (None)),
    "state_StopLine": (
        (trans_checkEmergency,trans_intersectionFound),
        ("cameraLines","cameraCorners","bumper","ultrasonic"),
        (None),
        (None)), 
    "state_IdentifyIntersection": (
        (trans_checkEmergency,trans_foundLeft90Turn,trans_foundDirectionalDevice,trans_foundYieldLeft,trans_foundStraightThrough),
        ("cameraLines", "cameraCorners","bumper","cameraColors","ultrasonic"),
        (None),
        (None)),
    "state_YieldtoLeft": (
        (trans_checkEmergency,trans_yieldLeftThenForeward,trans_yieldLefttoTurnRight),
        ("cameraLines","cameraCorners","bumper","ultrasonic"),
        (None),
        (None)),
    "state_DetermineLight": (
        (trans_checkEmergency,trans_lightTypeDetermined),
        ("cameraLines","cameraCorners","bumper","cameraColors","ultrasonic"),
        (None),
        (None)),
    "state_ExecuteTurn": (
        (trans_checkEmergency,trans_turnFinished,trans_returnSafeStop),
        ("cameraLines","cameraCorners","bumper","cameraColors","ultrasonic","colorSensors"),
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

def createDelay(customTime):
    global endTime,isDelayOver,startTime
    isDelayOver = False

    print("New delay has begun for stateMachine")
    startTime = time.monotonic_ns()
    if customTime is not None:
        endTime = time.monotonic_ns() + customTime
    else:
        endTime = time.monotonic_ns() + interStDelay

def checkDelay():
    global isDelayOver,endTime
    currentTime = time.monotonic()

    if (currentTime >= endTime):
        print ("State machine delay is now over!")
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

    #please note, running cameraCorners and cameraColors as a function intermittently is BAD CODE,
    #however, we may not have time to multithread depending on how long general debugging takes.
    #appologies, hope for the best...

    global isStopLineDetected, isObstaclePresent, isLineVisible, isCollisionBumperEngaged, foundSharpTurn
    global isInterTypeIdentified, isTrafficLightDetected, foundPathForward, foundLeftStopLine, foundRightStopLine, isLightDirectionDetermined, turnDirection
    
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
            case "cameraLines":
                #THIS IS UNUSED, BUT SAVED INCASE WE NEED TO DO CAMERA LINE TRACKING
                pass
            case "cameraCorners":
                cam.detectStopLines()
                sensorData.interpretCamera_Corners()
            case "cameraColors":
                cam.detectLEDS()
                sensorData.interpretCamera_Colors()
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

    isInterTypeIdentified = sensorData.intersectionTypeIdentified
    isTrafficLightDetected = sensorData.trafficLightDetected
    foundPathForward = sensorData.foundPathForward
    foundLeftStopLine = sensorData.foundLeftStopLine
    foundRightStopLine = sensorData.foundRightStopLine
    isLightDirectionDetermined = sensorData.lightDirectionDetermined
    turnDirection = sensorData.turnDirection #String direction for which way to turn

def checkForChangeStates():
    global stateIsTransitioning

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
    createDelay()
    sensorData.cameraLineTrackingEnabled = True
    gpio.resetGimbal()
    print("Line following will now start...")

    while(not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

        gpio.calculateBiasPD()
        gpio.updateMotion("pd")

def state_StopLine():
    global involvedInIntersection
    involvedInIntersection = True
    createDelay(1000000000)
    gpio.halt()
    print("STOP LINE HAS BEEN FOUND")

    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

def state_IdentifyIntersection():
    global involvedInIntersection
    involvedInIntersection = True
    createDelay(100)
    print("Attempting to identify intersection...")

    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

def state_YieldtoLeft():
    global involvedInIntersection
    involvedInIntersection = True
    createDelay(1000000000)
    print("Currently yielding to the left...")

    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

def state_DetermineLight():
    global involvedInIntersection
    involvedInIntersection = True
    createDelay(1000000000)
    print("Attempting to determine traffic light...")

    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()

def state_ExecuteTurn():
    global involvedInIntersection
    involvedInIntersection = True
    gpio.halt()
    gpio.moveIncremental('f',0.2)
    createDelay()
    print("Now attempting turn procedure...")

    while(sensorData.): #move up to intersection phase
        fetchSensorData()
        gpio.moveIncremental('f',0.01)
    while(): #rotation phase
        fetchSensorData()
        if(turnDirection == "right"):
            gpio.moveIncremental('cnclk',0.01)
        else:
            gpio.moveIncremental('cnclk',0.01)
    while (not stateIsTransitioning):
        checkDelay()
        fetchSensorData()
        checkForChangeStates()
    #reset camera gimbal
    involvedInIntersection = False

def state_Emergency():
    gpio.fntRed.on() #red LED on
    gpio.fntGreen.off() #green LED off
    print("BUMPER WAS ACTIVATED, CONTROL TERMINATED!")
    gpio.emergencyStop() #emergency stop motors <- ENTERS INFINITE LOOP

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