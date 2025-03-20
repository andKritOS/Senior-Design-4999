#---------GPIO INTERFACE CODE---------
#-------------------------------------
#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)s
#WRITTEN BY ANDREW KRITIKOS COPYRIGHT 2025

import gpiozero as gpio #gpio handling library
import board #PI gpio access
import sensorData
import adafruit_blinka #circuit python compatability for RGB sensor libraries
import adafruit_tcs34725 #RGB sensors
import adafruit_tca9548a #multiplexer
import time

#--------------------[[USER EDITABLE VARIABLES]]-------------------------------

#class  variables
_ultrasonicThreshDist = 0.3 #float (meters) of the distance when the vehicle should stop behind an obstacle
_servoMaxTurnAngle = 45 #maximum allowed angle (degrees) (positive and negative) for servo to turn.
_servoCalibrationAngle = 0 #angle (degrees) offset for which servo will return upon leaving
_motorMaxSpeed = 3 #speed at which the robot moves forward at full power (meters/sec).
_MotorIncIntervalSeconds = 0.05 #the amount of time (seconds) with which the incremental motion is to wait before checking total distance

#------------[[NON-EDITABLE VARIABLES, DO NOT EDIT PAST THIS LINE]]-------------

#-----PD control variables
biasMax = 100 
biasMin = 0
_kp = 0.2 #proportional control variable for pd control
_kd = 0.1 #differential control variable for pd control

#-----PD internal varaibles
_pd_new_err = 0 #current error value for pd control
_pd_old_err = 0 #previous error value for pd control
_timer_L = 0 #timer value for left
_timer_R = 0 #timer value for right
_TOB_L = 0 #time on blue, RIGHT SENSOR
_TOB_R = 0 #time on blue, RIGHT SENSOR

#--------------------[[USER EDITABLE VARIABLES]]--------------------------------

# ALL WILL BE SET TO OUTPUTS
# ONCE CONNECTED, PINS NEED TO BE CHANGED IN SOFTWARE
# FORMAT: (PIN NUM(#), IN(0)/OUT(1), PWM (1)/NOT PWM(0))multiThreadFunc

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
    #------------------COLOR_SENSOR------------------
    #declares I2C communication
    "I2CReset": (10,0,0),
    "I2C_SDA": (22,1,0),
    "I2C_SCL": (24,0,0),
    #------------------ULTRASONIC--------------------
    "ultraTrig": (26,1,0), #ultrasonic trigger pin
    "ultraEcho": (16,0,0) #ultrasonic feedback pin
}

#------------------------------SENSOR DECLARATIONS---------------------------------------------------

#FRONT LEFT
motorFL = gpio.Motor(
    pinAsgn["frontLeftForward"][0], #forward
    pinAsgn["frontLeftBackward"][0], #backward
)
#FRONT RIGHT
motorFR = gpio.Motor(
    pinAsgn["frontRightForward"][0], #forward
    pinAsgn["frontRightBackward"][0], #backward
)
#BACK LEFT
motorBL = gpio.Motor(
    pinAsgn["backLeftForward"][0], #forward
    pinAsgn["backLeftBackward"][0], #backward
)
#BACK RIGHT
motorBR = gpio.Motor(
    pinAsgn["backRightForward"][0], #forward
    pinAsgn["backRightBackward"][0], #backward
)
#CAMERA SERVO
camServo = gpio.AngularServoservo(
    pinAsgn["cameraGimbalServo"][0]
)

#-----FRONT RED LED
fntRed = gpio.DigitalOutputDevice(pinAsgn["frontRGB_Red"[0]])
#-----FRONT GREEN LED
fntGreen = gpio.DigitalOutputDevice(pinAsgn["frontRGB_Green"[0]])

#-----BUMPER SWITCHES
#Left Bumper Switch
bumperSWL = gpio.Button(pinAsgn["leftBumper"[0]])
#Center Bumper Switch
bumperSWC = gpio.Button(pinAsgn["centerBumper"[0]])
#Right Bumper Switch
bumperSWR = gpio.Button(pinAsgn["rightBumper"[0]])

#----I2C Connection
i2c = board.I2C() #creates I2C bus 
tca = adafruit_tcs34725.TCS34725(i2c) #creates a singular device

#-----Color Sensors
#create color sensor objects
rgbLeft = adafruit_tcs34725.TCS34725(tca[0])
rgbCenter = adafruit_tcs34725.TCS34725(tca[1])
rgbRight = adafruit_tcs34725.TCS34725(tca[2])

#-----Ultrasonic Sensor
ultSon = gpio.DistanceSensor(
    pinAsgn["ultraEcho"][0],
    pinAsgn["ultraTrig"][0]
)

# -------------------------------PD CONTROL FUNCTIONS--------------------------------

def percToSpd(speed):
    #speed is taken as percentage and converted into 0 to 1 PWM value
    speed = speed/100
    return speed

def boundValue(value):
    return max(biasMin, min(value, biasMax))

def findTOB():
    #check clocks
    time.time()

def calculateBiasPD():

    findTOB()
    E_now_L = 0 - _TOB_L
    E_now_R = 0 - _TOB_R


    bias = 50 - _kp*[()+()] + _kd*[()+()]
    return boundValue(bias)

# -------------------------------BASIC SENSOR POLL FUNCTIONS-----------------------------------

def pollBumpers():
    sensorData.sensorData["bumperSWL"][0] = bumperSWL.value
    sensorData.sensorData["bumperSWC"][1] = bumperSWC.value
    sensorData.sensorData["bumperSWR"][2] = bumperSWR.value
    print("Polling Bumpers \n")

def pollUltrasonic(position):
    #UNFINISHED
    #UNFINISHED
    #UNFINISHED
    #UNFINISHED
    sensorData.sensorData["ultSonic"][position] = ultSon
    print("Polling Ultrasonic Sensor \n")


def pollColorSens():
    sensorData.sensorData["colorSens"][0] = rgbLeft.color_rgb_bytes
    sensorData.sensorData["colorSens"][1] = rgbLeft.color_rgb_bytes
    sensorData.sensorData["colorSens"][2] = rgbLeft.color_rgb_bytes

# -------------------------------COMPOUND FUNCTIONS---------------------------------
def halt():
    #stops all motion
    motorFL.stop()
    motorFR.stop()
    motorBL.stop()
    motorBR.stop()
    print("Halting Motors \n")
    
def updateMotion(mode,speedPrcnt):
    
    #bias works on a scale of 0 to 100
    # 0 is directing all forward motion to the left motors
    # 100 is directing all forward motion to the right motorsspeedPrcnt
    # 50 is direction all forward motion evenly to both sets

    leftBiasSpeed = 0
    rightBiasSpeed = 0

    if speedPrcnt is None:
        #if no speed provided, default to full power
        speedPrcnt = 100
    else:
        leftBiasSpeed = speedPrcnt
        rightBiasSpeed = speedPrcnt

    match mode:
        case 'pd': #PID controlled   
            bias = calculateBiasPD()
            leftBiasSpeed = speedPrcnt * (100 - bias)/100
            rightBiasSpeed = speedPrcnt * (bias)/100

            motorFL.forward(speed = percToSpd(leftBiasSpeed))
            motorFR.forward(speed = percToSpd(rightBiasSpeed))
            motorBL.forward(speed = percToSpd(leftBiasSpeed))
            motorBR.forward(speed = percToSpd(rightBiasSpeed))
        case 'cw': #clockwise
            motorFL.forward(speed = percToSpd(speedPrcnt))
            motorFR.backward(speed = percToSpd(speedPrcnt))
            motorBL.forward(speed = percToSpd(speedPrcnt))
            motorBR.backward(speed = percToSpd(speedPrcnt))
        case 'ccw': #anticlockwise
            motorFL.backward(speed = percToSpd(speedPrcnt))
            motorFR.forward(speed = percToSpd(speedPrcnt))
            motorBL.backward(speed = percToSpd(speedPrcnt))
            motorBR.forward(speed = percToSpd(speedPrcnt))
        case _:
            print("ERROR INVALID DIRECTION ON INCREMENTAL MOVEMENT FUNCTION")
            distanceMeters = 0

def moveForDist(distanceMeters):
    while(distanceMoved < distanceMeters):
        #d = v * t
        time.sleep(_MotorIncIntervalSeconds) #delays time in seconds
        distanceMoved += ((_motorMaxSpeed * time.speedPrcnt)) * (_MotorIncIntervalSeconds)

def resetGimbal():
    camServo.value = _servoCalibrationAngle
    print("Camera gimbal has been reset.\n")

def lookBothWays():
    resetGimbal()
    time.sleep(0.1)
    camServo.value(-_servoMaxTurnAngle)
    time.sleep(0.1)    
    pollUltrasonic(0) #polls when angle is turned left
    time.sleep(0.1)
    camServo.value(_servoCalibrationAngle)
    time.sleep(0.1)
    pollUltrasonic(1) #polls when angle is centered
    time.sleep(0.1)
    camServo.value(_servoMaxTurnAngle)
    time.sleep(0.1)
    pollUltrasonic(2) #polls when angle is centered
    print("Finished looking both ways. \n")

def emergencyStop():
    #stops and then disconnects power from motors, program wont operate again until reset
    halt()
    motorFL.close()
    motorFR.close()
    motorBL.close()
    motorBR.close()
    camServo.value = None #will be able to freely move
    print("EMERGENCY STOP HAS BEEN DEPLOYED!\n")
    while(1):
        pass
        #THIS IS A DEAD END, CODE STOPS HERE UNTIL RESET
    
def moonWalk():
    print("WALKIN' ON THE MOON, BABY!\n")
    #turns wheels inward and accomplishes nothing to test motors
    motorFL.backward()
    motorFR.backward()
    motorBL.forward()
    motorBR.forward()
    for i in range(10):
        camServo.value = -0.5
        time.sleep(1)
        camServo.value = 0.5
        time.sleep(1)
    halt()