import cv2 as cv
import numpy as np
import os
from colorama import Fore, Back, Style, init

init() #initializes colorama for colored terminal text
os.chdir('/home/coldware/Documents/Oakland/4999Code/auxFiles') #ensures checking correct path for image test files

#stop line detection variables
dotsOnLeft = 0
dotsOnRight = 0
foundStopLineLeft = False
foundStopLineRight = False
foundPathForward = False
foundDirectionalDevice = False

path_linePhoto1 = 'LinePhoto1.jpg'
linePhoto1 = cv.imread(path_linePhoto1)
path_linePhoto2 = 'LinePhoto2.jpg'
linePhoto2 = cv.imread(path_linePhoto2)
path_linePhoto3 = 'LinePhoto3.jpg'
linePhoto3 = cv.imread(path_linePhoto3)

#camera feed settings
cameraWidth = 480
cameraHeight = 640
cameraChannelCnt = 3

#camera feed settings
#cameraWidth, cameraHeight, cameraChannelCnt = linePhoto1.shape

#HSV color definitions

hsvColors = {
"blueLo" : np.array([58, 0, 0]),
"blueHi" : np.array([125, 255, 255]),
"yellowHi" : np.array([30, 118, 255]),
"greenLo" : np.array([38, 28, 173]),
"greenHi" : np.array([79, 255, 255])
}

#create regions of interest
# creates region left most vertical third of the screen 
x_scrn, y_scrn, h_scrn = 0, 0, cameraHeight
w_tri_1 = round(cameraWidth * (1/3))
w_tri_2 = round(cameraWidth * (2/3)) #creates region center vertical third of the screen 
w_tri_3 = round(cameraWidth) #creates region right most vertical third of the screen 
#roiScreen = frameResized[y:yr+hr, x:xr+wr]

def startCameraCapture(channel):

     cap = cv.VideoCapture(channel)
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

def checkWithinROI(cornerImage):
        
        global dotsOnLeft,dotsOnRight

        for corner in corners:
            x, y = corner.ravel()
            frameFilterCorners = cv.circle(frameResized, center=(x,y), radius = 8, color=(0,0,255), thickness=-1)

            if ((x >= (x_scrn)) and (x <= (x_scrn + w_tri_1)) and (y >= (y_scrn)) and (y <= (y_scrn+h_scrn))): #checks for left third of the screen
               dotsOnLeft += 1
            
            if ((x >= (x_scrn + w_tri_2)) and (x <= (x_scrn + w_tri_3)) and (y >= (y_scrn)) and (y <= (y_scrn+h_scrn))): #checks for left third of the screen
                dotsOnRight += 1

        return frameFilterCorners

def drawROI(baseFrame):
    semiBase = cv.rectangle(baseFrame,(x_scrn,y_scrn),(x_scrn + w_tri_1,y_scrn + h_scrn),(0,255,0),thickness = 4)
    baseFrame = cv.rectangle(semiBase,(x_scrn + w_tri_2,y_scrn),(x_scrn + w_tri_3,y_scrn + h_scrn),(0,255,0),thickness = 4)
    return baseFrame

def drawBoxesForColor(baseFrame,maskFrame,boxText):
    cntFrame, _ = cv.findContours(maskFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in cntFrame:
        (x,y,w,h) = cv.boundingRect(cnt)
        cv.rectangle(baseFrame, (x,y), (x + w, y + h), (0, 255, 255), 3)
        cv.putText(baseFrame, boxText, (x, y-10), cv.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    return cntFrame
     
while True:
        frameRAW = linePhoto1

        frameResized = cv.resize(frameRAW,(cameraWidth,cameraHeight))
        frameExample = cv.resize(frameRAW,(cameraWidth,cameraHeight))

        #frameGray = cv.cvtColor(frameResized,cv.COLOR_BGR2GRAY)
        frameHSV = cv.cvtColor(frameResized,cv.COLOR_BGR2HSV)
        frameBlue = createHSVMasks(frameHSV,"blue")
        blurFrame = cv.GaussianBlur(frameBlue,(21,21),3)
        detectedEdges = cv.Canny(blurFrame,50,180)

        corners = cv.goodFeaturesToTrack(detectedEdges, maxCorners = 100, qualityLevel = 0.02, minDistance=10,useHarrisDetector=True,k=0.1)
        corners = np.int0(corners)
        frameFilterCorners = checkWithinROI(corners)
        frameFilterCorners = drawROI(frameFilterCorners)

        #to identify stop lines, I first need to isolate all lines that are associated with the color blue

        cv.imshow("BaseFrame", frameExample)
        cv.imshow("BlueFrame", frameBlue)
        cv.imshow("BlurFrame", blurFrame)
        cv.imshow("EdgeFrame", detectedEdges)
        cv.imshow("Corners", frameFilterCorners)

        key = cv.waitKey(1)
        if key == 27:
            break

os.system('clear')
if (dotsOnLeft >= 4):
    print(Fore.CYAN + "LEFT STOPLINE WAS DETECTED" + Style.RESET_ALL)
if (dotsOnRight >= 4):
    print(Fore.CYAN + "RIGHT STOPLINE WAS DETECTED" + Style.RESET_ALL)

cap.release()
cv.destroyAllWindows()