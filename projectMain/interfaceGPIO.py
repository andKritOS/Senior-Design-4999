import gpiozero as gpio
import board
import adafruit_tcs34725
from time import sleep

#--------------------[[USER EDITABLE VARIABLES]]-------------------------------

#class  variables
_hardwarePWMFrequency = 200
_softwarePWMFrequency = 200
_servoMaxTurnAngle = 90 #maximum allowed angle (degrees) (positive and negative) for servo to turn.
_servoCalibrationAngle = 0 #angle (degrees) offset for which servo will return upon leaving
_motorMaxSpeed = 3 #speed at which the robot moves forward at full power (meters/sec).
_maxRev = 90 #units of revolutions per minute
_MotorIncIntervalSeconds = 0.05 #the amount of time (seconds) with which the incremental motion is to wait before checking total distance
wheelRadius = 3.175 #wheel radius in cm
travelSpeed = (_maxRev*(2 * 3.14 * wheelRadius)/100)/60 #units in meters per second
ctrl_PD = {'P': 1,'D': 1} #PD calibration for smooth motor control


#------------[[NON-EDITABLE VARIABLES, DO NOT EDIT PAST THIS LINE]]-------------

sensorData = {
    "ultSonic":[0,0,0], #ultrasonic latest readings [Left,Center,Right]
    "colorSens":[[0,0,0], [0,0,0], [0,0,0]], # [L[RGB], C[RGB], R[RGB]]
    "bumper": 0 #if this ever becomes 1, something went wrong and the robot will stop
}

#--------------------[[USER EDITABLE VARIABLES]]--------------------------------

# ALL WILL BE SET TO OUTPUTS
# ONCE CONNECTED, PINS NEED TO BE CHANGED IN SOFTWARE
# FORMAT: (PIN NUM(#), IN(0)/OUT(1), PWM (1)/NOT PWM(0))


pinAsgn = {
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
    #declares I2C communication
    "Select_A0": (18,1,0), #please review, this needs to be an I2C connection because it doesn't strictly contain data pins
    "Select_A1": (23,0,0), 
    "Select_A2": (27,1,0),
    "I2CReset": (10,0,0),
    "I2C_SDA": (22,1,0),
    "I2C_SCL": (24,0,0),
    #------------------ULTRASONIC--------------------
    "ultraTrig": (2,1,0), #ultrasonic trigger pin
    "ultraEcho": (3,0,0) #ultrasonic feedback pin
}

# pin assignments
#FRONT LEFT
motorFL = gpio.Motor(
    pinAsgn["frontLeftForward"[0]], #forward
    pinAsgn["frontLeftBackward"[0]], #backward
    pinAsgn["frontLeftPWM"[0]], #backward
    True,
    None
)

#FRONT RIGHT
motorFR = gpio.Motor(
    pinAsgn["frontRightForward"[0]], #forward
    pinAsgn["frontRightBackward"[0]], #backward
    pinAsgn["frontRightPWM"[0]], #backward
    True,
    None
)

#BACK LEFT
motorBL = gpio.Motor(
    pinAsgn["backLeftForward"[0]], #forward
    pinAsgn["backLeftBackward"[0]], #backward
    pinAsgn["backLeftPWM"[0]], #backward
    True,
    None
)

#BACK RIGHT
motorBR = gpio.Motor(
    pinAsgn["backRightForward"[0]], #forward
    pinAsgn["backRightBackward"[0]], #backward
    pinAsgn["backRightPWM"[0]], #backward
    True,
    None
)

#CAMERA SERVO
camServo = gpio.AngularServoservo(
    pinAsgn["cameraGimbalServo"[0]],
    _servoMaxTurnAngle, #angle on reset
    -_servoCalibrationAngle, #min servo angle
    _servoCalibrationAngle, #max servo angle
    0.01, #min pw
    1, #max pw
    0.020, #frame width
    None
)

#FRONT RED LED
fntRed = gpio.LED(pinAsgn["frontRGB_Red"[0]])

#FRONT GREEN LED
fntGreen = gpio.LED(pinAsgn["frontRGB_Green"[0]])

#Bumper Switch
bumperSW = gpio.Button(
    pinAsgn["frontBumper"[0]],
    False, #False = pulldown resistor
    True, #True = Active High
    None, #no bounce time because the bumper should NEVER be touched
    1, #time to wait until executing "when held"
    False, #No hold repeat
    None #Pin factory, used for SPI
)

#Color sensors

#Left
clrSens_L = gpio.InputDevice()

#Center
clrSens_Cnt = adafruit_tcs34725.TCS34725()

#Right
clrSens_R = gpio.InputDevice()

#Ultrasonic Sensor
ultSon = gpio.DistanceSensor(
    pinAsgn["ultraTrig"[0]],
    pinAsgn["ultraEcho"[0]],
    9, #length of queue of read valuespinAsgn
    4.0, #max readable distance (meters)
    0.4, #IMPORTANT! THRESHOLD DISTANCE! TRIGGER IN RANGE DISTANCE FOR SENSOR
    False, #FALSE = report values ONLY after the queue has filled up
    None #pin factory
)


# -------------------------------I2C SETUP FUNCTIONS------------------------------------- 

# -------------------------------MOTOR COMPOUND FUNCTIONS------------------------------------- 
def percToSpd(speed):
    #speed is taken as percentage and converted into 0 to 1 PWM value
    speed = speed/100
    return speed

def halt():
    #stops all motion
    motorFL.stop()
    motorFR.stop()
    motorBL.stop()
    motorBR.stop()
    
def moveOrTurnIncremental(direction,distanceMeters,speedPrcnt):
    
    distanceMoved = 0

    if speedPrcnt is None:
        #if no speed provided, default to full power
        speedPrcnt = 100

    match direction:
        case 'f':
            motorFL.forward(speed = percToSpd(speedPrcnt))
            motorFR.forward(speed = percToSpd(speedPrcnt))
            motorBL.forward(speed = percToSpd(speedPrcnt))
            motorBR.forward(speed = percToSpd(speedPrcnt))
        case 'b':
            motorFL.backward(speed = percToSpd(speedPrcnt))
            motorFR.backward(speed = percToSpd(speedPrcnt))
            motorBL.backward(speed = percToSpd(speedPrcnt))
            motorBR.backward(speed = percToSpd(speedPrcnt))
        case 'clk':
            motorFL.forward(speed = percToSpd(speedPrcnt))
            motorFR.backward(speed = percToSpd(speedPrcnt))
            motorBL.forward(speed = percToSpd(speedPrcnt))
            motorBR.backward(speed = percToSpd(speedPrcnt))
        case 'cntr_clk':
            motorFL.backward(speed = percToSpd(speedPrcnt))
            motorFR.forward(speed = percToSpd(speedPrcnt))
            motorBL.backward(speed = percToSpd(speedPrcnt))
            motorBR.forward(speed = percToSpd(speedPrcnt))
        case _:
            print("ERROR INVALID DIRECTION ON INCREMENTAL MOVEMENT FUNCTION")
            distanceMeters = 0

    while(distanceMoved < distanceMeters):
        #d = v * t
        sleep(_MotorIncIntervalSeconds) #delays time in seconds
        distanceMoved += ((travelSpeed * speedPrcnt)) * (_MotorIncIntervalSeconds)

    halt()

#turns left or right incrementally
def followContinuous():
    
    motorFR.forward() 

def resetGimbal():
    camServo.

def lookBothWays():
    camServo.value()

def emergencyStop():
    #stops and then disconnects power from motors, program wont operate again until reset
    halt()

    motorFL.close()
    motorFR.close()
    motorBL.close()
    motorBR.close()
    camServo.value = None #will be able to freely move
    
def moonWalk():
    #turns wheels inward and accomplishes nothing to test motors
    motorFL.backward()
    motorFR.backward()
    motorBL.forward()
    motorBR.forward()
    for i in range(10):
        camServo.value = -0.5
        sleep(1)
        camServo.value = 0.5
        sleep(1)
    halt()




