import cv2 as cv
import numpy as np

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

def makeBWImage(baseFrame):
     newImage = cv.cvtColor(baseFrame,cv.COLOR_RGB2GRAY)
     return newImage

def createHSVMasks(baseFrame,frameColorName):
     newImage = cv.inRange(baseFrame, hsvColors[frameColorName + "Lo"], hsvColors[frameColorName + "Hi"])
     return newImage

def apply_AND_Mask(baseFrame,maskFrame):
    newImage = cv.bitwise_and(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def findHSVCircles(grayScaleFrame):
    colorFrame = cv.cvtColor(grayScaleFrame,cv.COLOR_GRAY2BGR)
    blurFrame = cv.medianBlur(colorFrame,6)
    circlesFrame = cv.HoughCircles(blurFrame,cv.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
    circlesFrame = np.unint16(np.around(circlesFrame))
    for i in circlesFrame[0, :]:
         cv.circle(colorFrame,(i[0],i[1]),i[2],(255,0,0),2) #draws a red circle around the circle
         cv.circle(colorFrame,(i[0],i[1]),2,(0,0,255),3) #draws a blue dot in the center of the circle
    return circlesFrame

def drawBoxesForColor(baseFrame,maskFrame,boxText):
    cntFrame, _ = cv.findContours(maskFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in cntFrame:
        (x,y,w,h) = cv.boundingRect(cnt)
        cv.rectangle(baseFrame, (x,y), (x + w, y + h), (0, 255, 255), 3)
        cv.putText(baseFrame, boxText, (x, y-10), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    return cntFrame

def extractColoredLEDs(baseRAWframe,colors,):
    funcBW = makeBWImage(baseRAWframe)
    funcCircles = findHSVCircles(funcBW)
    for i in colors:
         funcImage = apply_AND_Mask(baseRAWframe,funcCircles)

         
     
     
     
while True:
        frameRAW = startCameraCapture()
        frameHSV = makeHSVImage(frameRAW)
        frameBW = makeBWImage(frameRAW)

        

        #to create masks, type the color in lowercase, don't worry about low and high values, they will be added later
        greenMask = createHSVMasks(frameHSV,"green")

        #apply masks
        compGreen = apply_AND_Mask(frameRAW, greenMask)

        #draw box colors
        circleImage = findHSVCircles(frameBW)

        cv.imshow("Camera Feed", frameRAW)
        cv.imshow("Just Green",greenMask)
        cv.imshow("Final Composite Green",compGreen)


        key = cv.waitKey(1)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()