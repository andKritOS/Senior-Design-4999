import cv2 as cv
import p
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

     
while True:
        frameRAW = startCameraCapture() #frameRaw is BGR
        hsvFrame = cv.cvtColor(frameRAW,cv.COLOR_BGR2HSV) #hsvFrame is HSV
        grayScaleFrame = cv.cvtColor(frameRAW,cv.COLOR_BGR2GRAY) #grayScaleFrame is Gray

        colorHSVMask = createHSVMasks(hsvFrame,"GREEN") #Green HSV mask
        blurryFrame = cv.medianBlur(grayScaleFrame,5) # only takes 3 and 5 as kernel size when using uint8
        circlesMask = cv.HoughCircles(blurryFrame,cv.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0) #ciclesMask is gray

        if circlesMask is not None:
            circlesRings = np.uint8(np.around(circlesMask)) #circles rings is gray
            for i in circlesRings[0, :]:
                cv.circle(frameRAW,(i[0],i[1]),i[2],(255,0,0),2) #draws a red circle around the circle
                cv.circle(frameRAW,(i[0],i[1]),2,(0,0,255),3) #draws a blue dot in the center of the circle

            circlesRings = cv.cvtColor(circlesRings,cv.COLOR_RGB2HSV) #HSV
        
        circlesMask = cv.cvtColor(circlesMask,cv.COLOR_GRAY2BGR) #BGR
        colorHSVMask = cv.cvtColor(colorHSVMask,cv.COLOR_HSV2BGR) #BGRcirclesMask

        detectedLEDMask = apply_AND_Mask(colorHSVMask,circlesMask) #apply bit mask down to each of the detected circles
        finalOutput = apply_AND_Mask(frameRAW,detectedLEDMask)
        finalOutput = apply_OR_Mask(finalOutput,circlesRings)

        cv.imshow("Camera Feed", frameRAW)
        cv.imshow("Camera with Identified LEDs", finalOutput)

        key = cv.waitKey(1)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()