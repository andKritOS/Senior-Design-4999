import cv2 as cv
import numpy as np
import string

#camera feed settings
cameraWidth = 640
cameraHeight = 480
cameraChannelCnt = 3
thresh_value = 200

#HSV color definitions

hsvColors = {
"yellowLo" : np.array([14, 0, 255]),
"yellowHi" : np.array([30, 118, 255]),
"greenLo" : np.array([38, 28, 173]),
"greenHi" : np.array([79, 255, 255])
}

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

cap = cv.VideoCapture(0)#frameRaw is BGR

camera_width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
camera_height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
camera_channel_count = cap.get(cv.CAP_PROP_VIDEO_TOTAL_CHANNELS)

while True:
        ret, frameRAW = cap.read(0)
        frameRAWCircles = frameRAW

        maskGreenHSV = cv.cvtColor(frameRAW,cv.COLOR_BGR2HSV)
        maskGreenHSV = createHSVMasks(frameRAW,"green") #Green HSV mask
        maskYellowHSV = cv.cvtColor(frameRAW,cv.COLOR_BGR2HSV)
        maskYellowHSV = createHSVMasks(frameRAW,"yellow") #Green HSV mask
        
        baseFrameHSV = cv.cvtColor(frameRAW,cv.COLOR_BGR2HSV) #HSV
        baseFrameGray = cv.cvtColor(frameRAW,cv.COLOR_BGR2GRAY) #GRAY

        #creates a blurry frame with which to apply to the circles
        blurryFrame = cv.medianBlur(baseFrameGray, 5) # HSV only takes 3 and 5 as kernel size when using uint8
        #filters all brightest pixels from the screen given a certain threshold
        blurryFrame = cv.threshold(blurryFrame,thresh_value,255,cv.THRESH_BINARY)
        #GRAY ciclesMask is JUST WHITE DOTS OF CIRCLES
        maskCircles = cv.HoughCircles(blurryFrame,cv.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=100)

        if maskCircles is not None:
            maskCircles = np.uint8(np.around(maskCircles)) #converts from decimal to integer for all radii

            for i in maskCircles[0, :]:
                cv.circle(frameRAWCircles,(i[0],i[1]),i[2],(255,0,0),2) #draws a red circle around the circle
                cv.circle(frameRAWCircles,(i[0],i[1]),2,(0,0,255),3) #draws a blue dot in the center of the circle

        else:
            maskCircles = np.zeros((cameraHeight,cameraWidth,cameraChannelCnt), dtype = np.uint8) 

        maskCirclesHSV = cv.cvtColor(maskCirclesHSV,cv.COLOR_GRAY2BGR)
        maskCirclesHSV = cv.cvtColor(maskCirclesHSV,cv.COLOR_BGR2HSV)
        
        detectedLEDMask = apply_AND_Mask(maskGreenHSV,maskCirclesHSV) #apply bit mask down to each of the detected circles
        finalOutput = apply_AND_Mask(frameRAW,detectedLEDMask)
        #finalOutput = apply_OR_Mask(finalOutput,circlesRings)

        cv.imshow("Mask Green HSV", maskGreenHSV)
        cv.imshow("Blurry Frame Gray", blurryMask)
        cv.imshow("frame RAW Circles", frameRAWCircles)
        cv.imshow("mask Circles Gray", maskCircles)

        key = cv.waitKey(1)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()