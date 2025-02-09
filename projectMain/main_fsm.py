import camera_comp
import motor_comp
import sensors_lights

class Transition:
    def __init__(self,toState):
        self.toState = toState

    def Execute(self):
        # to add things here
        print("Transitioning now")

class templateFSM:

    def __init__(self,char):
        self.char = char
        self.states = {} #the states available
        self.transitions = {} #list of next states
        self.currentSt = None
        self.nextSt = None

    def setCurrState(self,stateName):
        #sets the current state, creates instance
        self.currentSt = self.states[stateName]

    def setNextState(self,transName):
        #sets the next state to go to, creates instance
        self.nextSt = self.transitions[transName]

    def Execute(self):
        if(self.nextSt):
            self.nextSt.Execute()
            self.SetState(self.nextSt.t)
            self.trans = None
        self.currentSt.Execute()

#constructing main FSM
class 
