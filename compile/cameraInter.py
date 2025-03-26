import cv2 as cv
import sensorData
import numpy as np
import string
from colorama import Fore, Back, Style, init

init() #initializes colorama for colored terminal text
#----------------------PRIVATE DEFINED VARIABLES--------------------------------
#camera feed settings
cameraWidth = 640
cameraHeight = 480
cameraChannelCnt = 3
thresh_value = 200
stopLineDotThresh = 4 # The number of corners the camera needs to give a postive result
stopLineTimeThresh = 2 # Time (sec) provided to "debounce" camera results

#-----stop line detection variables
dotsOnLeft = 0
dotsOnRight = 0

#-----Screen sector variables
x_scrn, y_scrn, h_scrn = 0, 0, cameraHeight
#cuts the image into thirds verticaly
w_tri_1 = round(cameraWidth * (1/3))
w_tri_2 = round(cameraWidth * (2/3)) 
w_tri_3 = round(cameraWidth)
w_half = round(cameraWidth * (0.5))

#cuts the image into thirds horizontally
h_tri_1 = round(cameraHeight * (1/3))
h_tri_2 = round(cameraHeight * (2/3))
h_tri_3 = round(cameraHeight)
h_half = round(cameraHeight * (0.5))

#-----HSV color definitions
hsvColors = {
    "yellowLo" : np.array([0, 27, 255]),
    "yellowHi" : np.array([30, 255, 255]),
    "greenLo" : np.array([38, 28, 173]),
    "greenHi" : np.array([79, 255, 255])
}

#-----Video capture declaration

cap = cv.VideoCapture(0) #frameRaw is BGR
cameraWidth = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
cameraHeight = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
camera_channel_count = cap.get(cv.CAP_PROP_VIDEO_TOTAL_CHANNELS)

#----------------------FRAME FUNCTIONS AND PROCESSING--------------------------------

def createHSVMasks(baseFrame,frameColorName):
     loCase = frameColorName.lower()
     newImage = cv.inRange(baseFrame, hsvColors[loCase + "Lo"], hsvColors[loCase + "Hi"])
     return newImage

def apply_AND_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_and(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def apply_OR_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_or(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def drawBoxesForColor(baseFrame,maskFrame,boxText):
    cntFrame, _ = cv.findContours(maskFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in cntFrame:
        (x,y,w,h) = cv.boundingRect(cnt)
        cv.rectangle(baseFrame, (x,y), (x + w, y + h), (0, 255, 255), 3)
        cv.putText(baseFrame, boxText, (x, y-10), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    return cntFrame

def drawCirclesForColor(baseFrame,maskFrame,boxText):
    cntFrame, _ = cv.findContours(maskFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in cntFrame:
        (x,y), radius = cv.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv.circle(baseFrame, center, radius, (0, 255, 255), 3)
        cv.putText(baseFrame, boxText, (x, y), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    return cntFrame

def checkWithinROI(cornerImage):
        
    global dotsOnLeft,dotsOnRight
    dotsOnLeft = 0
    dotsOnRight = 0

    #stop line detector
    for corner in corners:
        x, y = corner.ravel()
        frameFilterCorners = cv.circle(frameResized, center=(x,y), radius = 8, color=(0,0,255), thickness=-1)

        if ((x >= (x_scrn)) and (x <= (x_scrn + w_tri_1)) and (y >= (y_scrn)) and (y <= (y_scrn+h_scrn))): #checks for left third of the screen
            if (y >= (y_scrn + h_tri_1)) and (y <= (y_scrn + h_tri_2)):
                dotsOnLeft += 1
        
        if ((x >= (x_scrn + w_tri_2)) and (x <= (x_scrn + w_tri_3)) and (y >= (y_scrn)) and (y <= (y_scrn+h_scrn))): #checks for left third of the screen
            if (y >= (y_scrn + h_tri_1)) and (y <= (y_scrn + h_tri_2)):
                dotsOnRight += 1

    return frameFilterCorners

def drawROI(baseFrame):
    baseFrame = cv.rectangle(baseFrame,(x_scrn,y_scrn),(x_scrn + w_tri_1,y_scrn + h_scrn),(0,255,0),thickness = 4) #left vertical tri
    baseFrame = cv.rectangle(baseFrame,(x_scrn + w_tri_2,y_scrn),(x_scrn + w_tri_3,y_scrn + h_scrn),(0,255,0),thickness = 4) #right vertical tri
    baseFrame = cv.rectangle(baseFrame,(x_scrn,y_scrn+h_tri_1),(x_scrn + w_tri_3,y_scrn + h_tri_2),(0,0,255),thickness = 4) #middle horizontal tri
    return baseFrame

#------------ DATA MANAGEMENT FUNCTIONS

def resetCameraData():
    #resets values for when operations for a given cycle have completed
    sensorData.rightLightDetected = False
    sensorData.leftLightDetected = False
    sensorData.leftLightDetected = False
    sensorData.yellowLightDetected = False
    sensorData.rightLightDetected = False

    sensorData.cameraCornerTrackingEnabled = False
    sensorData.cameraColorTrackingEnabled = False

#------------------------MAJOR DETECTION FUNCTIONS--------------------------

def detectStopLines(cornerImage):
    sensorData.cameraCornerTrackingEnabled = True

    global corners,frameResized,frameFilterCorners
    ret,frameRAW = cap.read(0)

    frameResized = cv.resize(frameRAW,(cameraWidth,cameraHeight))

    frameHSV = cv.cvtColor(frameResized,cv.COLOR_BGR2HSV)
    frameBlue = createHSVMasks(frameHSV,"blue")
    blurFrame = cv.GaussianBlur(frameBlue,(51,51),3)
    detectedEdges = cv.Canny(blurFrame,50,180)
    detectedEdges = cv.GaussianBlur(detectedEdges,(5,5),3)

    corners = cv.goodFeaturesToTrack(detectedEdges, maxCorners = 100, qualityLevel = 0.02, minDistance=20.0,useHarrisDetector=True,k=0.1)
    if corners is not None:
        corners = np.int0(corners)
        frameFilterCorners = checkWithinROI(corners)
    else:
        frameFilterCorners = detectedEdges
    
    frameFilterCorners = drawROI(frameResized)

    #determines location of dots
    if (dotsOnLeft >= stopLineDotThresh):
        print(Fore.CYAN + "LEFT STOPLINE WAS DETECTED" + Style.RESET_ALL)
        sensorData.leftStopLineDetected = True
    elif (dotsOnRight >= stopLineDotThresh):
        print(Fore.CYAN + "RIGHT STOPLINE WAS DETECTED" + Style.RESET_ALL)
        sensorData.rightStopLineDetected = True
    else:
        print(Fore.RED + "NO STOPLINE WAS DETECTED" + Style.RESET_ALL)
        sensorData.leftStopLineDetected = False
        sensorData.rightStopLineDetected = False      

def detectLEDS():
    sensorData.cameraColorTrackingEnabled = True

    ret, frameRAW = cap.read()

    frameResized = cv.resize(frameRAW,(cameraWidth,cameraHeight))
    baseFrameGray = cv.cvtColor(frameResized,cv.COLOR_BGR2GRAY) #GRAY

    maskGreenHSV = cv.cvtColor(frameResized,cv.COLOR_BGR2HSV)
    maskYellowHSV = cv.cvtColor(frameResized,cv.COLOR_BGR2HSV)

    maskGreenHSV = createHSVMasks(maskGreenHSV,"green") #Green HSV mask
    maskYellowHSV = createHSVMasks(maskYellowHSV,"yellow") #Yellow HSV mask

    #creates a blurry frame with which to apply to the circles
    maskGreenHSV = cv.medianBlur(maskGreenHSV, 5) # HSV only takes 3 and 5 as kernel size when using uint8
    maskYellowHSV = cv.medianBlur(maskYellowHSV, 5) # HSV only takes 3 and 5 as kernel size when using uint8

    baseFrameGray = cv.medianBlur(baseFrameGray, 3) # HSV only takes 3 and 5 as kernel size when using uint8
    ret, maskBright = cv.threshold(baseFrameGray,210,255,cv.THRESH_BINARY) #filters all brightest pixels from the screen given a certain threshold

    for i in (maskGreenHSV,maskYellowHSV):
        cntFrame, _ = cv.findContours(i, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for cnt in cntFrame:
            (x,y), radius = cv.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            cv.circle(i, center, radius, (255, 255, 255), cv.FILLED)

    greenLEDPixels = apply_AND_Mask(maskGreenHSV,maskBright)
    yellowLEDPixels = apply_AND_Mask(maskYellowHSV,maskBright)

    #calculates the mean x position of the x pixels between the two photos
    totalArray = np.add(greenLEDPixels,yellowLEDPixels)
    major = cv.moments(totalArray)

    cX = 0
    if major["m00"] != 0:  # No division by zero
        cX = int(major["m10"] / major["m00"])
    else:
        cX = 0

    if(cX < w_half):
        sensorData.leftLightDetected = True
        print("Left light detected")
    elif(cX > w_half):
        print("Right light detected")
        sensorData.rightLightDetected = True
    else:
        print("Right on the money? \n")

    cv.imshow("greenLEDPixels", greenLEDPixels)
    cv.imshow("yellowLEDPixels", yellowLEDPixels)