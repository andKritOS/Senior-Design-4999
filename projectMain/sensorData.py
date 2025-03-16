#---------FUNCTIONS FOR FETCHING AND INTERPRETING SENSOR DATA---------
#---------------------------------------------------------------------
#WINTER 2025 SEMESTER 
#ME 4999 CAPSTONE PROJECT (GROUP 20)
#WRITTEN BY ANDREW KRITIKOS COPYRIGHT 2025

sensorData = {
    "ultSonic":[0,0,0], #ultrasonic latest readings [Left,Center,Right]
    "colorSens":[[0,0,0], [0,0,0], [0,0,0]], # [L[RGB], C[RGB], R[RGB]]
    "bumpers": [0,0,0] #if this ever becomes 1, something went wrong and the robot will stop
}

def interperetCamera():


#list out all camera frame types as individual variables
cameraFrame_RAW


def interperetBumpers():
    if (sensorData["bumpers"][0] or [1] or [2] == 1):
        return 1
    el: 
        return 0

def interpretSonicSensor():
    if (sensorData["ultSonic"[]] == ):
        
