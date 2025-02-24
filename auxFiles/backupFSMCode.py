import camera_comp
import interfaceGPIO
import 
import time
class Transition:
    def __init__(self,toState):
        self.toState = toState

    def Execute(self):
        # to add things here
        print("Transitioning now")

class FSM:

    def __init__(self,char,states,transitions, inital):
        self.char = char
        self.states = states #the states available
        self.transitions = {} #list of next states
        self.currentSt = inital
        self.transCond = None

    def setCurrState(self,stateName):
        #sets the current state
        self.currentSt = self.states[stateName]

    def setTransition(self,transName):
        #sets the next state to go to
        self.transCond = self.transitions[transName]

    def Execute(self):
        if(self.transCond):
            self.transCond.Execute()
            self.SetState(self.nextSt)
            self.transCond = None
        self.currentSt.Execute()

class State:
    def __init__(self,FSM):
        self.FSM = FSM
        

#states tuple
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

initialState = "Reset"

transitions = {
    (): "",

}

fsm = FSM() #constructing main FSM