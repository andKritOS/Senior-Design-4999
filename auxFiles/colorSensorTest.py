#---------GPIO INTERFACE CODE---------
#-------------------------------------
#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)
#WRITTEN BY ANDREW KRITIKOS COPYRIGHT 2025

import gpiozero as gpio #gpio handling library
import board #PI gpio access
import adafruit_blinka #circuit python compatability for RGB sensor libraries
import adafruit_tcs34725 #RGB sensors
import adafruit_tca9548a #multiplexer
from time import sleep

#--------------------[[USER EDITABLE VARIABLES]]-------------------------------

#setup I2C connection
i2c = board.I2C() #creates I2C bus 
tca = adafruit_tcs34725.TCS34725(i2c) #creates a singular device

#create color sensor objects

rgbLeft = adafruit_tcs34725.TCS34725(tca[0])
rgbRight = adafruit_tcs34725.TCS34725(tca[1])

#class  variables
_hardwarePWMFrequency = 200
_softwarePWMFrequency = 200
_ultrasonicThreshDist = 0.3 #float (meters) of the distance when the vehicle should stop behind an obstacle
_servoMaxTurnAngle = 45 #maximum allowed angle (degrees) (positive and negative) for servo to turn.
_servoCalibrationAngle = 0 #angle (degrees) offset for which servo will return upon leaving
_motorMaxSpeed = 3 #speed at which the robot moves forward at full power (meters/sec).
_MotorIncIntervalSeconds = 0.05 #the amount of time (seconds) with which the incremental motion is to wait before checking total distance

#------------[[NON-EDITABLE VARIABLES, DO NOT EDIT PAST THIS LINE]]-------------

#--------------------[[USER EDITABLE VARIABLES]]--------------------------------

# ALL WILL BE SET TO OUTPUTS
# ONCE CONNECTED, PINS NEED TO BE CHANGED IN SOFTWARE
# FORMAT: (PIN NUM(#), IN(0)/OUT(1), PWM (1)/NOT PWM(0))


pinAsgn = {
    #---------------------MOTORS---------------------
    "cameraGimbalServo": (9,1,0),  # independent servo (SET UP PIN TO USE SOFTWARE PWM, ALL HARDWARE USED)
    #---------------------LEDS-----------------------
    "frontRGB_Red": (4,1,0), # 
    "frontRGB_Green": (17,1,0), #
    #"rearRedLED": (X,X), #unused, will be directly plugged into the 3.3 volt pinb
    #------------------COLOR_SENSOR------------------
    #declares I2C communication
    "Select_A0": (18,1,0), #please review, this needs to be an I2C connection because it doesn't strictly contain data pins
    "Select_A1": (23,0,0), 
    "Select_A2": (27,1,0),
    "I2CReset": (10,0,0),
    "I2C_SDA": (22,1,0),
    "I2C_SCL": (24,0,0),
    #------------------ULTRASONIC--------------------
    "ultraTrig": (26,1,0), #ultrasonic trigger pin
    "ultraEcho": (16,0,0) #ultrasonic feedback pin
}

# pin assignments

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

#Color sensors

#Left
clrSens_L = rgbLeft.color

#Right
clrSens_R = rgbRight.color

#Ultrasonic Sensor
ultSon = gpio.DistanceSensor(
    pinAsgn["ultraTrig"][0],
    pinAsgn["ultraEcho"][0]
    #9, #length of queue of read valuespinAsgn
    #4.0, #max readable distance (meters)
    #_ultrasonicThreshDist, #IMPORTANT! THRESHOLD DISTANCE! TRIGGER IN RANGE DISTANCE FOR SENSOR
    #False, #FALSE = report values ONLY after the queue has filled up
    #None #pin factory
)
# -------------------------------BASIC SENSOR POLL FUNCTIONS-----------------------------------

def pollUltrasonic(position):
    
    print (ultSon.value)
    print("Polling Ultrasonic Sensor \n")
# -------------------------------I2C SETUP FUNCTIONS------------------------------------- 

# -------------------------------COMPOUND FUNCTIONS---------------------------------

def resetGimbal():
    camServo.value = _servoCalibrationAngle
    print("Camera gimbal has been reset.\n")

def lookBothWays():
    resetGimbal()
    sleep(0.001)
    camServo.value(-_servoMaxTurnAngle)
    sleep(0.001)    
    pollUltrasonic(0) #polls when angle is turned left
    sleep(0.001)
    camServo.value(_servoCalibrationAngle)
    sleep(0.001)
    pollUltrasonic(1) #polls when angle is centered
    sleep(0.001)
    camServo.value(_servoMaxTurnAngle)
    sleep(0.001)
    pollUltrasonic(1) #polls when angle is centered
    print("Finished looking both ways. \n")

#program start

while(1):
    lookBothWays()
    sleep (1)
    print("Blinking Lights. \n")
    sleep (1)
    print("Left RGB Sensor: {}".format(clrSens_L), end = "/n")
    print("Right RGB Sensor: {}".format(clrSens_L), end = "/n")
