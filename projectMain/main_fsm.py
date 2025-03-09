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
lightDirection = None #String direction for which way to turn

#COMPOUND TRANSITION COMBINATIONS
trans_leaveReset = [True, "state_Reset"] #CONDITIONLESS, MEANT TO GO IMMEDIATELY
trans_checkEmergency = [(isCollisionBumperEngaged) and (True), "state_Safety_Stop"] #1
trans_stopLineDetected = [(isStopLineDetected),"state_StopLine"] #2
trans_loopSafeStop= [(isObstaclePresent) or (not isLineVisible) or (isCollisionBumperEngaged), "state_Emergency"] #3
trans_returnSafeStop = [(isObstaclePresent) or (not isLineVisible) or (isCollisionBumperEngaged), "state_Safety_Stop"] #4
trans_beginDriving = [(not isObstaclePresent) and (isLineVisible) and (not isCollisionBumperEngaged), "state_Emergency"] #5
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
        (trans_checkEmergency,trans_beginDriving, trans_checkEmergency),
        ("cameraLines","bumper","ultrasonic"),
        ()), #regular car stop, activates for obstacles or intermediate 
    "state_FollowingLine": (
        (trans_returnSafeStop,trans_stopLineDetected),
        ("cameraLines","cameraCorners","bumper","ultrasonic","colorRGB"),
        ()),
    "state_StopLine": (
        (trans_intersectionFound,trans_returnSafeStop),
        ("cameraLines","cameraCorners","bumper","ultrasonic"),
        ()), 
    "state_IdentifyIntersection": (
        (trans_foundLeft90Turn,trans_foundDirectionalDevice,trans_foundYieldLeft,trans_foundStraightThrough, trans_returnSafeStop),
        ("cameraLines","cameraCorners","cameraColors","ultrasonic"),
        ()),
    "state_YieldtoLeft": (
        (trans_yieldLeftThenForeward,trans_yieldLefttoTurnRight,trans_returnSafeStop),
        ("cameraLines","cameraCorners","ultrasonic"),
        ()),
    "state_DetermineLight": ((trans_lightTypeDetermined,trans_returnSafeStop),
        ("cameraLines","cameraCorners","cameraColors","ultrasonic"),
        ()),
    "state_ExecuteTurn": (
        (trans_turnFinished,trans_returnSafeStop),
        ("cameraLines","cameraCorners","cameraColors","ultrasonic"),
        ()),
    "state_Emergency": (
        (None),
        (None),
        ())
}

#--------------------------SECONDARY CONTROL-FLOW FUNCTIONS------------------------

def fetchSensorData():

    for i in stateNames[currentState[1]]:
        match currentState[1]:
            case "cameraLines":
                if not cameraLineTrackingEnabled:
                    cameraLineTrackingEnabled = True
                sensorData.interpretCameraLines()
            case "cameraDirectDev":
                if not cameraDirectDevEnabled:
                    cameraDirectDevEnabled = True
                sensorData.interpretCameraColors()
            case "cameraCorners":
                if not cameraCornerTrackingEnabled:
                    cameraCornerTrackingEnabled = True
                sensorData.interpretCameraCorners()
            case "bumper":
                gpio.pollBumpers()
                sensorData.interperetBumpers()
            case "ultrasonic":
                if (currentState == ""):
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

def checkForChangeStates(name):

    for i in stateNames[currentState[0]]:
        if (i == True):
            nextState = i[1]

    if (nextState is not None):
        currentState = nextState
        nextState = None #resets back to unchanged

def resetAllVars():
    #reset all variables
    currentState = "state_reset"
    lightDirection = "Left"
    nextState = None
    interStDelay = 1
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
    delay() #wait 1 second
    initalize() #sets up sensors, lights, etc.
    resetAllVars() #reset all functions
    checkForChangeStates()


def state_Safety_Stop():
    #delay
    delay()
    #turn motors off
    gpio.emergencyStop()
    while(1):
        pass
    #wait for transition conditions

def state_FollowingLine():
    #center gimbal
    gpio.resetGimbal()
    while(not trans_stopLineDetected):
        #START MULTITHREAD
        #while Sensor checks do not indicate stopLine and until intersection is found -> stop
        #run line following routine 
        #loop checking for obstacles until stop line is detected
            #check if line is present -> idle
            #check if obstacle is present -> idle
            #check if collision was engaged -> idle

def state_IdentifyIntersection():
    #delay
    delay(4)
    while (not trans_goToIdentifyFeatures)
    #loop until one is found below
        #wait for traffic lights nearby -> determine traffic lights (turn left or right)
        #wait for stop lines on left -> yield to left (make turn right)
        #wait for stop lines on right -> turn (make turn left)
        #wait for path ahead -> following Line

def state_StopLine():
    delay(2)
    while (not )
    #stop state

def state_YieldtoLeft(): 
    #delay
    delay()
    #check if car is present on left
        #loop wait until car has passed
    #loop check left and right until both ways are clear
    #if intersection goes straight -> following line
    #if intersection goes right -> turn (right)

def state_DetermineLight():
    #delay
    delay()
    #

def state_ExecuteTurn():
    #delay
    delay()
    #reset camera gimbal
    
    # loop until line is on track again AND visible in camera
    #

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
                
    print ("FSM HAS LEFT THE CONTROL LOOP")

#--------------------[PROGRAM INITIALIZATION]----------------------------

#initialize sensors 
initalize() #initialize basic systems
ctrlLoop() #run FSM and rest of program

    
    

