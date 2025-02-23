import camera_comp
import interfaceGPIO
import time

class ctrlVars:

    def __init__(self):

        #GLOBAL VARIABLES
        self.currentState = 
        self.nextState = None
        self.interStDelay = 1 #time to delay by default between states

        #STATE CONDITIONAL BOOLEANS
        self.isStopLineDetected = False
        self.isObstaclePresent = False
        self.isLineVisible = False
        self.isCollisionBumperEngaged = False
        self.foundSharpTurn = False
        self.isTurnComplete = False
        self.isDelayOver = True
        self.isInterTypeIdentified = False
        self.isTrafficLightDetected = False
        self.foundPathForward = False
        self.foundLeftStopLine = False
        self.foundRightStopLine = False
        self.isLightDirectionDetermined = False
        self.lightDirection = "Left"

        #COMPOUND TRANSITION COMBINATIONS
        self.trans_leaveReset = [True, states.stateReset] #CONDITIONLESS, MEANT TO GO IMMEDIATELY
        self.trans_checkEmergency = [(self.isCollisionBumperEngaged) and (True), states.state_Idle_Stop] #1
        self.trans_loopIdle = [(self.isObstaclePresent) or (not self.isLineVisible) or (self.isCollisionBumperEngaged), states.state_Emergency] #3
        self.trans_beginDriving = [(not self.isObstaclePresent) and (self.isLineVisible) and (not self.isCollisionBumperEngaged), states.state_Emergency] #5
        self.trans_returnIdle = [(self.isObstaclePresent) or (not self.isLineVisible) or (self.isCollisionBumperEngaged), states.state_Idle_Stop] #4
        self.trans_foundStraightThrough = [(self.isInterTypeIdentified) and (not self.isObstaclePresent) and (self.foundPathForward) and (self.isDelayOver) and (not self.foundLeftStopLine) and (self.foundRightStopLine),states.state_FollowingLine] #7
        self.trans_yieldLeftThenForeward = [(not self.isObstaclePresent) and (self.foundPathForward) and (self.isDelayOver), states.state_FollowingLine] #8
        self.trans_intersectionFound = [(not self.isObstaclePresent) and (self.isDelayOver), states.state_IdentifyIntersection] #11
        self.trans_foundLeft90Turn = [(self.isInterTypeIdentified) and (not self.foundPathForward) and (not self.foundLeftStopLine) and (self.foundRightStopLine),states.state_ExecuteTurn] #12
        self.trans_foundDirectionalDevice = [(self.isInterTypeIdentified) and (self.isTrafficLightDetected),states.state_DetermineLight] #13
        self.trans_foundYieldLeft = [(self.isInterTypeIdentified) and (self.foundLeftStopLine) and (not self.foundRightStopLine), states.state_YieldtoLeft] #14
        self.trans_yieldLefttoTurnRight = [(not self.isObstaclePresent) and (not self.foundPathForward),states.state_ExecuteTurn] #16
        self.trans_turnNotFinished = [(not self.isTurnComplete) or (not self.isDelayOver),states.state_ExecuteTurn] #17

    #----------STATE FUNCTION DEFINITIONS---------
    
    self.stateNames = {
        "state_Reset": {
            "Trans":(self.trans_leaveReset),
            "Func": stateReset }, #Initial state, sets global variables, initializes camera and sensors
        "state_Idle_Stop": (trans_checkEmergency,self.trans_loopIdle,), #regular car stop, activates for obstacles or intermediate 
        "state_FollowingLine": (),
        "state_Stop": (), 
        "state_IdentifyIntersection": (),
        "state_YieldtoLeft": (),
        "state_DetermineLight": (),
        "state_ExecuteTurn": (),
        "state_Emergency": () #bumper activation, stops car, changes lights, disconnects motors
    }

class states:

    #--------FLOW FUNCTION DEFINITIONS--------------

    def delay(self,customTime):
        #may need to consider timestamp approach if this holds up code
        if customTime is not None:

            time.sleep(customTime)
        else:
            time.sleep(ctrlVars.interStDelay)

                

    def state_Reset(self):
        #delay
        #reset all variables
        #initialize sensors and motors
        #center gimbal
        #startCamera feed
        #wait for transition conditions ->

    def state_Idle_Stop(self):
        #delay
        #turn motors off
        #wait for transition conditions

    def state_FollowingLine(self):
        #delay
        #center gimbal
        #loop until intersection is found -> stop
            #run line following routine 
            #loop checking for obstacles until stop line is detected
                #check if line is present -> idle
                #check if obstacle is present -> idle
                #check if collision was engaged -> idle


    def state_IdentifyIntersection(self):
        #delay
        #loop until one is found below
            #wait for traffic lights nearby -> determine traffic lights (turn left or right)
            #wait for stop lines on left -> yield to left (make turn right)
            #wait for stop lines on right -> turn (make turn left)
            #wait for path ahead -> following Line

    def state_YieldtoLeft(self): 
        #delay
        #check if car is present on left
            #loop wait until car has passed
        #loop check left and right until both ways are clear
        #if intersection goes straight -> following line
        #if intersection goes right -> turn (right)


    def state_DetermineLight(self):
        #delay
        #
    
    def state_ExecuteTurn(self):
        #delay
        #reset camera gimbal
        # loop until line is on track again AND visible in camera
        #

        
    def state_Emergency(self):
        #change LED to red
        #loop indefinitely
            #run emergency motor code immediately
        #wait for turn complete -> following line

    def changeStates(self):


    def ctrlLoop(self):
        #THIS CASE STATEMENT NEEDS TO EXIST INSTEAD OF JUST CHAINING BETWEEN FUNCTIONS.
        #WE CAN'T CHAIN BECAUSE PYTHON HAS A RECURSION DEPTH OF 1000, AND WE WILL EVENTUALLY RUN OUT OF MEMORY
        #RECURSION HAPPENS WHEN YOU RUN A FUNCTION FROM INSIDE A FUNCTION.

        while(1): #Primary loop
            if (ctrlVars.nextState is not None):
                match ctrlVars.nextState:
                    case "Reset":
                        state_Reset()
                    case "Idle_Stop":
                    case "FollowingLine":
                    case "Stop":
            else:
                



ctrlVars = ctrlVars #initializes FSM and control
states = states #initializes state structure
states.ctrlLoop() #run FSM

    
    

