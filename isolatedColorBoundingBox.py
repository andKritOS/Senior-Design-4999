import cv2 as cv
import numpy as np

from PIL import Image

cap = cv.VideoCapture(0)
video_filter = cv.createBackgroundSubtractorMOG2()

#HSV color definitions (pure)
yellowLED_HSV = [([39,63], [19, 255], [216,255]),
([26,31], [0, 104], [232,255])] #HSV yellow then green [H,S,V](min, max)

while True:
        ret, frame = cap.read()
        maskhsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
        mask = cv.inRange(maskhsv, )

        for (lower, upper) in 

        lowLim, upLim = correctHSV()

        cv.imshow("TEST_VIDEO_FEED", frame)
        cv.imshow("Mask",mask)

        key = cv.waitKey(30)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()