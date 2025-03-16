#------PROGRAM STATE MACHINE CODE-------
#---------------------------------------
#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)
#WRITTEN BY ANDREW KRITIKOS COPYRIGHT 2025

import multiThreadFunc as threading
import sensorData
import interfaceGPIO as gpio 
import time

#GLOBAL VARIABLES
currentState = "state_reset"
nextState = None
interStDelay = 1 #default time to delay by default between states
intrSlack = 0.20 #distance (meters) between the stop line and the intersection

#GLOBAL CONDITIONAL BOOLEANS
cameraDirectDevEnabled = False #enables directional device detecting for camera 
cameraLineTrackingEnabled = False #enables line and forward path verification for camera
cameraCornerTrackingEnabled = False #enables stop line detection for camera

#LOCAL CONDITIONAL BOOLEANS
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

#INTERSTATE CONDITIONS
stateIsTransitioning = False
involvedInIntersection = False

#COMPOUND TRANSITION COMBINATIONS
trans_leaveReset = [True, "state_Reset"] #CONDITIONLESS, MEANT TO GO IMMEDIATELY
trans_returnTurn = [(involvedInIntersection) and (not isObstaclePresent) and (isLineVisible) and (not isCollisionBumperEngaged), "state_ExecuteTurn"] #path is taken incase turn is interrupted by obstacle
trans_checkEmergency = [(isCollisionBumperEngaged) and (True), "state_Safety_Stop"] #1
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
        # Transition Conidtions [0] <- These are the conditions that are searched through to determine if the state should change
        # Requisite Sensor Groups [1] <- These are sensors or parts of camera code that are searched through so the state only has to refresh important data
        # Transition Variable Changes [2] <- These are variables that need to be reset AFTER a state transition happens

    "state_Reset": (
        (trans_leaveReset, trans_returnSafeStop),
        ("cameraLines","ultrasonic","colorRGB"),
        (None)),
    "state_Safety_Stop": (
        (trans_checkEmergency,trans_beginDriving, trans_returnTurn),
        ("cameraLines","bumper","ultrasonic"),
        (None)), #regular car stop, activates for obstacles or intermediate 
    "state_FollowingLine": (
        (trans_returnSafeStop,trans_stopLineDetected),
        ("cameraLines","cameraCorners","bumper","ultrasonic","colorRGB"),
        (None)),
    "state_StopLine": (
        (trans_intersectionFound,trans_returnSafeStop),
        ("cameraLines","cameraCorners","bumper","ultrasonic"),
        (None)), 
    "state_IdentifyIntersection": (
        (trans_foundLeft90Turn,trans_foundDirectionalDevice,trans_foundYieldLeft,trans_foundStraightThrough, trans_returnSafeStop),
        ("cameraLines","cameraCorners","cameraColors","ultrasonic"),
        (None)),
    "state_YieldtoLeft": (
        (trans_yieldLeftThenForeward,trans_yieldLefttoTurnRight,trans_returnSafeStop),
        ("cameraLines","cameraCorners","ultrasonic"),
        (None)),
    "state_DetermineLight": (
        (trans_lightTypeDetermined,trans_returnSafeStop),
        ("cameraLines","cameraCorners","cameraColors","ultrasonic"),
        (None)),
    "state_ExecuteTurn": (
        (trans_turnFinished,trans_returnSafeStop),
        ("cameraLines","cameraCorners","cameraColors","ultrasonic"),
        (None)),
    "state_Emergency": (
        (None),
        (None),
        (None))
}

#--------------------------SECONDARY CONTROL-FLOW FUNCTIONS------------------------

def resetStateVars():
    #reset all variables
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

def fetchSensorData():

    codeVal = 0

    for i in stateNames[currentState][1]:
        match currentState[1]:
            case "bumper":
                gpio.pollBumpers()
                codeVal = sensorData.interperetBumpers()
                if (codeVal == ) 
            case "ultrasonic":
                if (currentState == "state_IdentifyIntersection" or "state_YieldtoLeft"):
                    gpio.lookBothWays()
                else:
                    gpio.pollUltrasonic()
                sensorData.intepretSonicSensor()
            case "colorRGB":
                gpio.checkColor
                sensorData.interpretColorSensor()
            case None:
                pass
            case _:
                raise Exception("INVALID SENSOR WAS ACCESSED")
        
def checkForChangeStates(loopingState):

    for i in stateNames[currentState[0]]:
        if (i == True):
            nextState = i[1]

    if (nextState is not None):
        resetStateVars() #resets the required values for the state that was last exited
        currentState = nextState
        nextState = None #resets back to unchanged
        stateIsTransitioning = True        

def delay(customTime):
    isDelayOver = False
    #may need to consider timestamp approach if this holds up code
    if customTime is not None:
        time.sleep(customTime)
    else:
        time.sleep(interStDelay)
    isDelayOver = True

def initalize():
    gpio.fntRed.off()
    gpio.fntGreen.on()
    gpio.resetGimbal()

#----------------------------------STATE FUNCTIONS-----------------------------------------

def state_Reset():
    #delay() #wait 1 second
    initalize() #sets up sensors, lights, etc.
    resetStateVars() #reset all functions
    while(not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()

def state_Safety_Stop():
    #delay()
    #turn motors off
    gpio.emergencyStop()
    while(not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()
    #wait for transition conditions

def state_FollowingLine():
    #center gimbal
    #START MULTITHREAD
    #while Sensor checks do not indicate stopLine and until intersection is found -> stop
    #run line following routine 
    #loop checking for obstacles until stop line is detected
        #check if line is present -> idle
        #check if obstacle is present -> idle
        #check if collision was engaged -> idle
    cameraLineTrackingEnabled = True
    gpio.resetGimbal()
    while(not stateIsTransitioning):
        gpio.followContinuous()
        fetchSensorData()
        checkForChangeStates()

def state_IdentifyIntersection():
    #delay
    #delay(4)
    involvedInIntersection = True
    while (not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()
        #loop until one is found below
        #wait for traffic lights nearby -> determine traffic lights (turn left or right)
        #wait for stop lines on left -> yield to left (make turn right)
        #wait for stop lines on right -> turn (make turn left)
        #wait for path ahead -> following Line

def state_StopLine():
    #delay(2)
    involvedInIntersection = True
    gpio.halt()
    while (not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()
        #stop state

def state_YieldtoLeft():
    #delay()
    involvedInIntersection = True
    while (not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()
    #check if car is present on left
        #loop wait until car has passed
    #loop check left and right until both ways are clear
    #if intersection goes straight -> following line
    #if intersection goes right -> turn (right)

def state_DetermineLight():
    #delay
    involvedInIntersection = True
    while (not stateIsTransitioning):
        fetchSensorData()
        checkForChangeStates()
    delay()

def state_ExecuteTurn():
    #delay
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

#-----------------------PRIMARY CONTROL-FLOW FUNCTIONS------------------------------------------------------------

def ctrlLoop():
    #THIS CASE STATEMENT NEEDS TO EXIST INSTEAD OF JUST CHAINING BETWEEN FUNCTIONS.
    #WE CAN'T CHAIN BECAUSE PYTHON HAS A RECURSION DEPTH OF 1000, AND WE WILL EVENTUALLY RUN OUT OF MEMORY
    #RECURSION HAPPENS WHEN YOU RUN A FUNCTION FROM INSIDE A FUNCTION.

    resetStateVars()
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
                
    print ("FSM HAS LEFT THE CONTROL LOOP")

#--------------------[PROGRAM INITIALIZATION]----------------------------

#initialize sensors 
initalize() #initialize basic systems
ctrlLoop() #run FSM and rest of program