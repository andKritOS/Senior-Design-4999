import cv2 as cv
import numpy
from PIL import Image
cap = cv.VideoCapture(0)

video_filter = cv.createBackgroundSubtractorMOG2()

while True:
        ret, frame = cap.read()
        maskhsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
        mask = cv.inRange(maskhsv, )

        cv.imshow("TEST_VIDEO_FEED", frame)
        cv.imshow("Mask",mask)

        key = cv.waitKey(30)
        if key == 27:
            break

cap.release()
cv.destroyAllWindows()