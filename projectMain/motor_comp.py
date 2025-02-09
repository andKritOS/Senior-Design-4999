import pigpio

#instances class that sets up GPIO communication to local pi
mainPi = pigpio.pi()

class pinSetup:
    def __init__(self):

class apply        

class moveOperations:
    def __init__(self):
    def moveIncremental(self,direction,distance,velocity):
        #moves forward or backward incrementally
    def turnIncrement(self,direction,degrees,angVelocity):
        #turns left or right incrementally
    def moveContinuous(self,direction,velocity):
        #moves forward or backward continuously
    def turnContinuous(self,direction,angVelocity):
        #moves forward or backward continuously
    def halt(self):
        #stops all motion
    def emergencyStop(self):
        #disconnects power from motors
    def moonWalk(self):
        #turns wheels inward and accomplishes nothing

class compoundOperations: