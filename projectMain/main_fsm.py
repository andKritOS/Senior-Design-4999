import camera_comp
import interfaceGPIO
import time

class states:

    def __init__(self):
        #GLOBAL VARIABLES
        self.isStopLineDetected = False
        self.isObstaclePresent = False
        self.isLineVisible = False
        self.isCollisionBumperEngaged = False
        self.isSharpTurn = False
        self.isTurnComplete = False
        self.isDelayOver = True
        self.isInterTypeIdentified = False
        self.pathForwardExists = False
        self.leftStopLineExists = False
        self.rightStopLineExists = False
        self.isLightDirectionDetermined = False
        self.lightDirection = "Left"

    #list of states tuple
    states = {
        "Reset", #Initial state, sets global variables, initializes camera and sensors
        "Idle_Stop", #regular car stop, activates for obstacles or intermediate 
        "FollowingLine",
        "Stop", 
        "IdentifyIntersection",
        "YieldtoLeft",
        "DetermineLight",
        "ExecuteTurn",
        "Emergency" #bumper activation, stops car, changes lights, disconnects motors
    }

    chkEmergency = self.isCollisionBumperEngaged & True

    transitions = {
        "chkEmergency": (self.isCollisionBumperEngaged),
        "chkObstacle": ()
    }



    def stateReset(self):
        #startCamera
        #

    def Idle_Stop(self):

    def FollowingLine(self):

    def IdentifyIntersection(self):

    def YieldtoLeft(self): 

    def DetermineLight(self): 
    
    def ExecuteTurn(self):  

    def Emergency(self):  

class fsmCtrl:
    
    def __init__(self):
        currentState = "Reset"
        initialState = "Reset"
        transition = None

    def updateVariables(self):

    
    def ctrlLoop(self):
        while(1): #Primary loop
            if ()

    
        
ctrl = fsmCtrl
ctrl.ctrlLoop()

    
    

