import pigpio

#instances class that sets up GPIO communication to local pi
mainPi = pigpio.pi()

class pinSetup:
    def __init__(self):

        # ALL WILL BE SEET TO OUTPUTS
        # ONCE CONNECTED, PINS NEED TO BE CHANGED IN SOFTWARE
        # FORMAT: (PIN NUM, IN/OUT)

        self.pins = {
            "frontLeftForward": (0,1),  # controller 1, MotorA, IN1
            "frontLeftBackward": (1,1),  # controller 1, MotorA, IN2
            "frontRightForward": (2,1),  # controller 1, MotorB, IN3
            "frontRightBackward": (3,1),  # controller 1, MotorB, IN4
            "frontLeftPWM": (4,1),  # controller 1, MotorB, ENA
            "frontRightPWM": (5,1),  # controller 1, MotorB, ENB
            "backLeftForward": (6,1),  # controller 2, MotorA, IN1
            "backLeftBackward": (7,1),  # controller 2, MotorA, IN2
            "backRightForward": (8,1),  # controller 2, MotorB, IN3
            "backRightBackward": (9,1),  # controller 2, MotorB, IN4
            "backLeftPWM": (10,1),  # controller 2, MotorB, ENA
            "backRightPWM": (11,1)  # controller 2, MotorB, ENB
        }

        for i in self.pins:
            if (i[1] == 1):
                mainPi.set_mode(i[0],pigpio.OUTPUT)
            else:
                mainPi.set_mode(i[0],pigpio.INPUT)

            
        
    
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
