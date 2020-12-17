# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils
import cv2
import time
# from picamera import PiCamera
# from picamera.array import PiRGBArray

class USBcamControl():
    def __init__(self):
        self.cap = cv2.VideoCapture(-1)

    def capture(self):
        self.ret, self.raw = self.cap.read()
        self.frame = np.array(self.raw)
        self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
        return self.frame

class point():
    def __init__(self, X, Y, ID):
        self.coordinates = [X, Y]
        self.ID = ID

def connectedComponentAnalysis(frame):
    # perform a connected component analysis on the thresholded
    # image, then initialize a mask to store only the "large"
    # components
    labels = measure.label(frame, background=0)
    mask = np.zeros(frame.shape, dtype="uint8")
    # loop over the unique components
    for label in np.unique(labels):
        # if this is the background label, ignore it
        if label == 0:
            continue
        # otherwise, construct the label mask and count the
        # number of pixels 
        labelMask = np.zeros(frame.shape, dtype="uint8")
        labelMask[labels == label] = 255
        numPixels = cv2.countNonZero(labelMask)
        # if the number of pixels in the component is sufficiently
        # large, then add it to our mask of "large blobs"
        if numPixels > 5:
            mask = cv2.add(mask, labelMask)
    return mask

def findContours(frame, mask):
    points = []
    # find the contours in the mask, then sort them from left to right
    Contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Contours = imutils.grab_contours(Contours)
    Contours = contours.sort_contours(Contours)[0]
    # loop over the contours
    for (i, c) in enumerate(Contours):
        # draw the bright spot on the image
        (x, y, w, h) = cv2.boundingRect(c)
        ((cX, cY), radius) = cv2.minEnclosingCircle(c)
        print(f"point {i} = [X:{cX};Y:{cY}]")
        if radius < 10:
            cv2.circle(frame, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
            cv2.putText(frame, "#{}".format(i), (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
    return frame
    

def findPoints(frame):
    # Convert to gray scale image & put an threshold on the image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

    mask = connectedComponentAnalysis(thresh)
    frame = findContours(thresh, mask)
    return frame


def main():
    cam           = USBcamControl()

    while True:
        frame = cam.capture()

        frame = findPoints(frame)

        cv2.imshow("frame", frame)

        #To be able to stop the programm
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): 
            break

if __name__ == "__main__":
    main()