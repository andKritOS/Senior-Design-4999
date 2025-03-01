import camera_comp as camera
import interfaceGPIO as gpio
import time

class ctrlVars:

    def __init__(self):

        #GLOBAL VARIABLES
        self.currentState = "state_reset"
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
        self.trans_leaveReset = [True, "state_Reset"] #CONDITIONLESS, MEANT TO GO IMMEDIATELY
        self.trans_checkEmergency = [(self.isCollisionBumperEngaged) and (True), "state_Idle_Stop"] #1
        self.trans_stopLineDetected = [(self.isStopLineDetected),"state_Stop"] #2
        self.trans_loopIdle = [(self.isObstaclePresent) or (not self.isLineVisible) or (self.isCollisionBumperEngaged), "state_Emergency"] #3
        self.trans_returnIdle = [(self.isObstaclePresent) or (not self.isLineVisible) or (self.isCollisionBumperEngaged), "state_Idle_Stop"] #4
        self.trans_beginDriving = [(not self.isObstaclePresent) and (self.isLineVisible) and (not self.isCollisionBumperEngaged), "state_Emergency"] #5
        self.trans_goToIdentifyFeatures = [((not self.isObstaclePresent) and (self.isDelayOver)),"state_Stop"]
        self.trans_foundStraightThrough = [(self.isInterTypeIdentified) and (not self.isObstaclePresent) and (self.foundPathForward) and (self.isDelayOver) and (not self.foundLeftStopLine) and (self.foundRightStopLine),"state_FollowingLine"] #7
        self.trans_yieldLeftThenForeward = [(not self.isObstaclePresent) and (self.foundPathForward) and (self.isDelayOver), "state_FollowingLine"] #8
        self.trans_intersectionFound = [(not self.isObstaclePresent) and (self.isDelayOver), "state_IdentifyIntersection"] #11
        self.trans_foundLeft90Turn = [(self.isInterTypeIdentified) and (not self.foundPathForward) and (not self.foundLeftStopLine) and (self.foundRightStopLine),"state_ExecuteTurn"] #12
        self.trans_foundDirectionalDevice = [(self.isInterTypeIdentified) and (self.isTrafficLightDetected),"state_DetermineLight"] #13
        self.trans_foundYieldLeft = [(self.isInterTypeIdentified) and (self.foundLeftStopLine) and (not self.foundRightStopLine), "state_YieldtoLeft"] #14
        self.trans_lightTypeDetermined = [(self.isLightDirectionDetermined),"state_ExecuteTurn"] 
        self.trans_yieldLefttoTurnRight = [(not self.isObstaclePresent) and (not self.foundPathForward),"state_ExecuteTurn"] #16        self.trans_returnIdle = [(self.isObstaclePresent) or (not self.isLineVisible) or (self.isCollisionBumperEngaged), states.state_Idle_Stop] #4
        self.trans_turnFinished = [(self.isTurnComplete) and (self.isDelayOver),"state_FollowingLine"] #17
        self.trans_sharpTurnEncountered = [(self.foundSharpTurn),"state_ExecuteTurn"]

        #----------STATE FUNCTION DEFINITIONS---------
    
        self.stateNames = {
            "state_Reset": (self.trans_leaveReset),
            "state_Idle_Stop": (self.trans_checkEmergency,self.trans_beginDriving), #regular car stop, activates for obstacles or intermediate 
            "state_FollowingLine": (self.trans_returnIdle,self.trans_stopLineDetected),
            "state_Stop": (self.trans_intersectionFound), 
            "state_IdentifyIntersection": (self.trans_foundLeft90Turn,self.trans_foundDirectionalDevice,self.trans_foundYieldLeft,self.trans_foundStraightThrough),
            "state_YieldtoLeft": (self.trans_yieldLeftThenForeward,self.trans_yieldLefttoTurnRight),
            "state_DetermineLight": (self.trans_lightTypeDetermined),
            "state_ExecuteTurn": (self.trans_turnFinished),
            "state_Emergency": (None) #bumper activation, stops car, changes lights, disconnects motors
        }

class states:

    #--------FLOW FUNCTION DEFINITIONS--------------

    def checkChangeStates(self,name):
        for i in ctrlVars.stateNames[ctrlVars.currentState]:
            if (i == True):
                ctrlVars.nextState = i[1]

    def resetAllFunctions(self):
        #reset all variables
        self = ctrlVars
        #initialize sensors and motors
        #center gimbal
        #startCamera feed

    def delay(self,customTime):
        ctrlVars.isDelayOver = False

        #may need to consider timestamp approach if this holds up code
        if customTime is not None:

            time.sleep(customTime)
        else:
            time.sleep(ctrlVars.interStDelay)
        
        ctrlVars.isDelayOver = True
                
    def state_Reset(self):
            #delay
            self.delay()
            #reset all functions
            self.resetAllFunctions()
            #wait for transition conditions ->

    def state_Idle_Stop(self):
        #delay
        self.delay()
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

    def state_Stop(self):
        #stop state

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

    def ctrlLoop(self):
        #THIS CASE STATEMENT NEEDS TO EXIST INSTEAD OF JUST CHAINING BETWEEN FUNCTIONS.
        #WE CAN'T CHAIN BECAUSE PYTHON HAS A RECURSION DEPTH OF 1000, AND WE WILL EVENTUALLY RUN OUT OF MEMORY
        #RECURSION HAPPENS WHEN YOU RUN A FUNCTION FROM INSIDE A FUNCTION.

        while(1): #Primary loop
            self.checkChangedStates()
            if (ctrlVars.nextState is not None):
                ctrlVars.currentState = 
                match ctrlVars.nextState:
                    case "state_Reset":
                        self.state_Reset()
                    case "state_Idle_Stop":
                        self.state_Idle_Stop()
                    case "state_FollowingLine":
                        self.state_FollowingLine()
                    case "state_Stop":
                        self.state_Stop()
                    case "state_IdentifyIntersection":
                        self.state_IdentifyIntersection()
                    case "state_YieldtoLeft":
                        self.state_YieldtoLeft()
                    case "state_DetermineLight":
                        self.state_DetermineLight()
                    case "state_ExecuteTurn":
                        self.state_ExecuteTurn()
                    case "state_Emergency":
                        self.state_Emergency()
                    case _:
                        raise Exception("INVALID STATE ID")
                    
                ctrlVars.nextState = None #resets back to unchanged
            else:
                pass

ctrlVars = ctrlVars #initializes FSM and control
states = states #initializes state structure
states.ctrlLoop() #run FSM

    
    

