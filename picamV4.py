# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils
import cv2
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

class camControl():
    def __init__(self, resolution = (960, 720), framerate = 15, rotation = 0):
        self.cam = PiCamera()
        self.cam.resolution = resolution
        self.cam.framerate = framerate
        self.cam.rotation = rotation

        self.rawCap = PiRGBArray(self.cam)
        time.sleep(0.1)

    def capture(self):
        self.cam.capture(self.rawCap, format="bgr")
        self.frame = np.array(self.rawCap.array)
        return self.frame

    def clear(self):
        self.rawCap.truncate(0)






def main():
    cam           = camControl()

    while True:
        frame = cam.capture()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)

        # threshold the image to reveal light regions in the
        # blurred image
        thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]

        # perform a series of erosions and dilations to remove
        # any small blobs of noise from the thresholded image
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)
        
        cv2.imshow("Thresh hold image", thresh)

        #To be able to stop the programm
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): 
            break

if __name__ == "__main__":
    main()