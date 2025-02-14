import cv2 as cv
import numpy as np
import string

cap = cv.VideoCapture(0)

#HSV color definitions

hsvColors = {
"yellowLo" : np.array([14, 0, 255]),
"yellowHi" : np.array([30, 118, 255]),
"greenLo" : np.array([38, 28, 173]),
"greenHi" : np.array([79, 255, 255])
}

def startCameraCapture():
     ret, frame = cap.read(0)
     return frame

def makeHSVImage(baseFrame):
     newImage = cv.cvtColor(baseFrame,cv.COLOR_RGB2HSV)
     return newImage

def makeGrayImage(baseFrame):
     newImage = cv.cvtColor(baseFrame,cv.COLOR_RGB2GRAY)
     return newImage

def createHSVMasks(baseFrame,frameColorName):
     loCase = string.lower(frameColorName)
     newImage = cv.inRange(baseFrame, hsvColors[loCase + "Lo"], hsvColors[loCase + "Hi"])
     return newImage

def apply_AND_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_and(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def apply_OR_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_or(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def findHSVCircles(baseFrame,returnOnlyMask):

    # maskOrOverlay (0 = returns mask image only, 1 = returns mask image and ring image)
    grayScaleFrame = makeGrayImage(baseFrame)
    BGRFrame = cv.cvtColor(grayScaleFrame,cv.COLOR_GRAY2BGR) #BGR
    blurBGRFrame = cv.medianBlur(BGRFrame,6) #BGR
    circlesMask = cv.HoughCircles(blurBGRFrame,cv.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0) #BGR

    if (returnOnlyMask):
        circlesOnly = np.unint16(np.around(circlesMask)) #BGR
        for i in circlesOnly[0, :]:
            cv.circle(BGRFrame,(i[0],i[1]),i[2],(255,0,0),2) #draws a red circle around the circle
            cv.circle(BGRFrame,(i[0],i[1]),2,(0,0,255),3) #draws a blue dot in the center of the circle
    else:
        circlesOnly = None
    
    circlesMask = cv.cvtColor(circlesMask,cv.COLOR_BGR2HSV) #HSV
    circlesRing = cv.cvtColor(circlesOnly,cv.COLOR_BGR2HSV) #HSV

    return circlesMask, circlesRing

def drawBoxesForColor(baseFrame,maskFrame,boxText):
    cntFrame, _ = cv.findContours(maskFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in cntFrame:
        (x,y,w,h) = cv.boundingRect(cnt)
        cv.rectangle(baseFrame, (x,y), (x + w, y + h), (0, 255, 255), 3)
        cv.putText(baseFrame, boxText, (x, y-10), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    return cntFrame

def extractColoredLEDs(baseRAWframe,colors,includeRings):
    #'colors' takes in a list of colored LEDs to be found and dispOp will display
    _layerBW = makeGrayImage(baseRAWframe) #first makes a black and white frame to use in the circles finder
    _layerCirclesMask,_layerCirclesRings = findHSVCircles(_layerBW, 1) #finds circles in frame
    for string in colors:
         
         _layerHSVMask = createHSVMasks(baseRAWframe,string) #apply color mask for each color listed in "colors"
         _layerDetectedLEDMask = apply_AND_Mask(_layerHSVMask,_layerCirclesMask) #apply bit mask down to each of the detected circles
         _finalOutput = apply_AND_Mask(baseRAWframe,_layerDetectedLEDMask)

         if (includeRings and _layerCirclesRings is not None):
            _finalOutput = apply_OR_Mask(_finalOutput,_layerCirclesRings)
        
    return _finalOutput      
     
while True:
        frameRAW = startCameraCapture()
        greenLEDs = extractColoredLEDs(frameRAW,"green",1)

        cv.imshow("Camera Feed", frameRAW)
        cv.imshow("Camera with Identified LEDs", greenLEDs)

        key = cv.waitKey(1)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()