import pigpio

#instances class that sets up GPIO communication to local pi
mainPi = pigpio.pi()

class gpio:
    def __init__(self):

        #class variables
        self.__hardwarePWMFrequency = 200
        self.__softwarePWMFrequency = 200
        self.wheelDutyCycle = 0.5
        self


        # ALL WILL BE SEET TO OUTPUTS
        # ONCE CONNECTED, PINS NEED TO BE CHANGED IN SOFTWARE
        # FORMAT: (PIN NUM(#), IN(0)/OUT(1), PWM (1)/NOT PWM(0))

        self._pinAssignment = {
            #---------------------MOTORS---------------------
            "frontLeftForward": (0,1,0),  # controller 1, MotorA, IN1
            "frontLeftBackward": (1,1,0),  # controller 1, MotorA, IN2
            "frontRightForward": (2,1,0),  # controller 1, MotorB, IN3
            "frontRightBackward": (3,1,0),  # controller 1, MotorB, IN4
            "frontLeftPWM": (4,1,1),  # controller 1, MotorB, ENA
            "frontRightPWM": (5,1,1),  # controller 1, MotorB, ENB
            "backLeftForward": (6,1,0),  # controller 2, MotorA, IN1
            "backLeftBackward": (7,1,0),  # controller 2, MotorA, IN2
            "backRightForward": (8,1,0),  # controller 2, MotorB, IN3
            "backRightBackward": (9,1,0),  # controller 2, MotorB, IN4
            "backLeftPWM": (10,1,1),  # controller 2, MotorB, ENA
            "backRightPWM": (11,1,1),  # controller 2, MotorB, ENB
            "cameraGimbalServo": (12,1,0),  # independent servo (SET UP PIN TO USE SOFTWARE PWM, ALL HARDWARE USED)
            #---------------------LEDS-----------------------
            "frontRGB_Red": (13,1,0), # 
            "frontRGB_Green": (14,1,0), #
            #"rearRedLED": (X,X), #unused, will be directly plugged into the 3.3 volt pinb
            #---------------------BUTTONS--------------------
            "frontBumper": (15,1,0),
            #------------------COLOR_SENSOR------------------
            "frontRGB_send": (16,1,0), 
            "frontRGB_recieve": (17,0),
            "leftRGB_send": (18,1,0),
            "leftRGB_recieve": (19,0,0),
            "rightRGB_send": (20,1,0),
            "rightRGB_recieve": (21,0,0),
            #------------------ULTRASONIC--------------------
            "ultraTrig": (22,1,0), #ultrasonic trigger pin
            "ultraEcho": (23,0,0) #ultrasonic feedback pin
        }

        for i in self.pinAssignment:
            #sets pin direction
            if (i[1] == 1):
                mainPi.set_mode(i[0],pigpio.OUTPUT)
            else:
                mainPi.set_mode(i[0],pigpio.INPUT)
            #sets PWM on or off
            if (i[2] == 1):
                mainPi.hardware_PWM(i[1],)

    def setPWM

    
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

class lightModes
    def __init__(self):


class compoundOperations:
    def _


mainPins = gpio()


