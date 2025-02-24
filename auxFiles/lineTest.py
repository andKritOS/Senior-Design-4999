import cv2 as cv
import numpy as np
import os

#camera feed settings
cameraWidth = 640
cameraHeight = 480
cameraChannelCnt = 3

path_linePhoto1 = 'LinePhoto1.jpg'
path_linePhoto2 = 'LinePhoto2.jpg'
path_linePhoto3 = 'LinePhoto3.jpg'


linePhoto1 = cv.imread(path_linePhoto1)
linePhoto2 = cv.imread(path_linePhoto2)
linePhoto3 = cv.imread(path_linePhoto3)

#HSV color definitions

hsvColors = {
"blueLo" : np.array([58, 0, 0]),
"blueHi" : np.array([125, 255, 255]),
"yellowHi" : np.array([30, 118, 255]),
"greenLo" : np.array([38, 28, 173]),
"greenHi" : np.array([79, 255, 255])
}

def startCameraCapture():
     ret, frame = cap.read(0)
     camera_width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
     camera_height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
     camera_channel_count = cap.get(cv.CAP_PROP_VIDEO_TOTAL_CHANNELS)

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
        frameRAW = linePhoto1
        frameResized = cv.resize(frameRAW,(480,640))
        #frameEmpty = np.zeros((480,640,3),np.uint8)

        frameGray = cv.cvtColor(frameResized,cv.COLOR_BGR2GRAY)
        blurFrame = cv.GaussianBlur(frameResized,(21,21),3)
        detectedEdges = cv.Canny(blurFrame,50,180)

        corners = cv.goodFeaturesToTrack(detectedEdges, maxCorners = 50, qualityLevel = 0.01, minDistance=50,useHarrisDetector=True,k=0.1)
        corners = np.int8(corners)

        for corner in corners:
            x, y = corner.ravel()
            frameFilterCorners = cv.circle(frameResized, center=(x,y), radius = 20, color=(0,0,255), thickness=-1)

        #to identify stop lines, I first need to isolate all lines that are associated with the color blue.
        

        baseFrameHSV = cv.cvtColor(frameResized,cv.COLOR_BGR2HSV) #HSV

        cv.imshow("BaseFrame", frameResized)
        cv.imshow("GrayFrame", frameGray)
        cv.imshow("BlurFrame", blurFrame)
        cv.imshow("EdgeFrame", detectedEdges)
        cv.imshow("Corners", frameFilterCorners)

        key = cv.waitKey(1)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()