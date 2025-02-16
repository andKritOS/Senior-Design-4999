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
            "frontLeftForward": (11,1,0),  # controller 1, MotorA, IN1_0
            "frontLeftBackward": (7,1,0),  # controller 1, MotorA, IN2_0
            "frontRightForward": (5,1,0),  # controller 1, MotorB, IN3_0
            "frontRightBackward": (6,1,0),  # controller 1, MotorB, IN4_0
            "frontLeftPWM": (11,1,1),  # controller 1, MotorB, ENA_0
            "frontRightPWM": (12,1,1),  # controller 1, MotorB, ENB_0
            "backLeftForward": (19,1,0),  # controller 2, MotorA, IN1_1
            "backLeftBackward": (16,1,0),  # controller 2, MotorA, IN2_1
            "backRightForward": (26,1,0),  # controller 2, MotorB, IN3_1
            "backRightBackward": (20,1,0),  # controller 2, MotorB, IN4_1
            "backLeftPWM": (13,1,1),  # controller 2, MotorB, ENA_1
            "backRightPWM": (21,1,1),  # controller 2, MotorB, ENB_1
            "cameraGimbalServo": (,1,0),  # independent servo (SET UP PIN TO USE SOFTWARE PWM, ALL HARDWARE USED)
            #---------------------LEDS-----------------------
            "frontRGB_Red": (4,1,0), # 
            "frontRGB_Green": (17,1,0), #
            #"rearRedLED": (X,X), #unused, will be directly plugged into the 3.3 volt pinb
            #---------------------BUTTONS--------------------
            "frontBumper": (,1,0),
            #------------------COLOR_SENSOR------------------
            "frontRGB_SDA": (18,1,0), #please review, this needs to be an I2C connection because it doesn't strictly contain data pins
            "frontRGB_SCL": (0,0), 
            "leftRGB_SDA": (,1,0),
            "leftRGB_SCL": (,0,0),
            "rightRGB_SDA": (,1,0),
            "rightRGB_SCL": (,0,0),
            #------------------ULTRASONIC--------------------
            "ultraTrig": (2,1,0), #ultrasonic trigger pin
            "ultraEcho": (3,0,0) #ultrasonic feedback pin
        }

        for i in self.pinAssignment:
            #sets pin direction
            if (i[1] == 1):
                mainPi.set_mode(i[0],pigpio.OUTPUT)
            else:
                mainPi.set_mode(i[0],pigpio.INPUT)
            #sets PWM on or off
            if (i[2] == 1):
                mainPi.hardware_PWM(i[0],)

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


