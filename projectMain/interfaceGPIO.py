import gpiozero as gpio
import time

class gpioInit:
    def __init__(self):

        #class variables
        self.__hardwarePWMFrequency = 200
        self.__softwarePWMFrequency = 200
        self.wheelDutyCycle = 0.5
        self


        # ALL WILL BE SEET TO OUTPUTS
        # ONCE CONNECTED, PINS NEED TO BE CHANGED IN SOFTWARE
        # FORMAT: (PIN NUM(#), IN(0)/OUT(1), PWM (1)/NOT PWM(0))

        self.pinAsgn = {
            #---------------------MOTORS---------------------
            "frontLeftForward": (8,1,0),  # controller 1, MotorA, IN1_0
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
            "cameraGimbalServo": (9,1,0),  # independent servo (SET UP PIN TO USE SOFTWARE PWM, ALL HARDWARE USED)
            #---------------------LEDS-----------------------
            "frontRGB_Red": (4,1,0), # 
            "frontRGB_Green": (17,1,0), #
            #"rearRedLED": (X,X), #unused, will be directly plugged into the 3.3 volt pinb
            #---------------------BUTTONS--------------------
            "frontBumper": (25,1,0),
            #------------------COLOR_SENSOR------------------
            "frontRGB_SDA": (18,1,0), #please review, this needs to be an I2C connection because it doesn't strictly contain data pins
            "frontRGB_SCL": (23,0,0), 
            "leftRGB_SDA": (27,1,0),
            "leftRGB_SCL": (10,0,0),
            "rightRGB_SDA": (22,1,0),
            "rightRGB_SCL": (24,0,0),
            #------------------ULTRASONIC--------------------
            "ultraTrig": (2,1,0), #ultrasonic trigger pin
            "ultraEcho": (3,0,0) #ultrasonic feedback pin
        }

        # pin assignments
        #FRONT LEFT
        motorFL = gpio.Motor(
            self.pinAsgn["frontLeftForward"[0]], #forward
            self.pinAsgn["frontLeftBackward"[0]], #backward
            self.pinAsgn["frontLeftPWM"[0]], #backward
            True,
            None
            )

        #FRONT RIGHT
        motorFR = gpio.Motor(
            self.pinAsgn["frontRightForward"[0]], #forward
            self.pinAsgn["frontRightBackward"[0]], #backward
            self.pinAsgn["frontRightPWM"[0]], #backward
            True,
            None
            )

        #BACK LEFT
        motorBL = gpio.Motor(
            self.pinAsgn["backLeftForward"[0]], #forward
            self.pinAsgn["backLeftBackward"[0]], #backward
            self.pinAsgn["backLeftPWM"[0]], #backward
            True,
            None
            )

        #BACK RIGHT
        motorBR = gpio.Motor(
            self.pinAsgn["backRightForward"[0]], #forward
            self.pinAsgn["backRightBackward"[0]], #backward
            self.pinAsgn["backRightPWM"[0]], #backward
            True,
            None
            )

        #CAMERA SERVO
        camServo = gpio.servo(
            self.pinAsgn["cameraGimbalServo"[0]],
            0, #initial value on startup
            0.01, #min pw
            1, #max pw
            0.020, #frame width
            None
        )

        #FRONT RED LED
        fntRed = gpio.LED(self.pinAsgn["frontRGB_Red"[0]])

        #FRONT GREEN LED
        fntRed = gpio.LED(self.pinAsgn["frontRGB_Green"[0]])

        #Bumper Switch
        bumperSW = gpio.Button(
            self.pinAsgn["frontBumper"[0]],
            False, #False = pulldown resistor
            True, #True = Active High
            None, #no bounce time because the bumper should NEVER be touched
            1, #time to wait until executing "when held"
            False, #No hold repeat
            None #Pin factory, used for SPI
            )
        
        #Color sensors
        #Left
        clrSens_L = 
        #Center
        clrSens_Cnt
        #Right
        clrSens_R

        #Ultrasonic Sensor
        ultSon = gpio.DistanceSensor(
            self.pinAsgn["ultraTrig"[0]],
            self.pinAsgn["ultraEcho"[0],
            9, #length of queue of read values
            4.0, #max readable distance (meters)
            0.4, #IMPORTANT! THRESHOLD DISTANCE! TRIGGER IN RANGE DISTANCE FOR SENSOR
            False, #FALSE = report values ONLY after the queue has filled up
            None #pin factory
            ])

        

    
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


