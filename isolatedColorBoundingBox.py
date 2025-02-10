import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)

#HSV color definitions

hsvColors = {
"yellowLo" : np.array([14, 0, 255]),
"yellowHi" : np.array([30, 118, 255]),
"greenLo" : np.array([32, 0, 200]),
"greenHi" : np.array([60, 255, 255])
}

def createMasks(base,frameColorName):
     newImage = cv.inRange(base, hsvColors[frameColorName + "Lo"], hsvColors[frameColorName + "Hi"])
     return newImage

def applyMask(baseFrame,maskFrame):
    newImage = cv.bitwise_and(baseFrame, baseFrame, mask= maskFrame)
    return newImage

def applyContours(baseFrame,maskFrame):
    newImage = cv.bitwise_and(baseFrame, baseFrame, mask= maskFrame)
    return newImage

while True:
        ret, frame = cap.read()
        frameHSV = cv.cvtColor(frame,cv.COLOR_RGB2HSV)

        #to create masks, type the color in lowercase, don't worry about low and high values, they will be added later
        #yellowMask = createMasks(frameHSV,"yellow")
        greenMask = createMasks(frameHSV,"green")

        #apply masks
        #compYellow = applyMask(frameHSV, yellowMask)
        compGreen = applyMask(frameHSV, greenMask)

        cntYellow, _ = cv.findContours(greenMask,)

        cv.imshow("Full Feed", frame)

        #cv.imshow("Just Yellow",yellowMask)
        #cv.imshow("Final Composite Yellow",compYellow)

        cv.imshow("Just Green",greenMask)
        cv.imshow("Final Composite Green",compGreen)


        key = cv.waitKey(1)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()