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

    def storeDist(self, pnt, dis):
        self.pnt = pnt
        self.dis = dis
        print(f"Point {self.ID} has stored pnt: {self.pnt}, with dis: {self.dis}")
    
####### Finding points
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
    Contours = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    Contours = imutils.grab_contours(Contours)
    Contours = contours.sort_contours(Contours)[0]
    # loop over the contours
    for (i, c) in enumerate(Contours):
        # draw the bright spot on the image
        (x, y, w, h) = cv2.boundingRect(c)
        ((cX, cY), radius) = cv2.minEnclosingCircle(c)
        if radius < 10:
            cv2.circle(frame, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
            cv2.putText(frame, "#{}".format(i), (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
            #Add point to list
            points.append(point(cX, cY, i))
    return frame, points
    
def findPoints(frame):
    # Convert to gray scale image & put an threshold on the image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

    mask = connectedComponentAnalysis(thresh)
    frame, points = findContours(frame, mask)
    return frame, points

######## Find triangles
def calculateDistances(points):
    for pnt in points:
        ID1 = pnt.ID
        x1 = pnt.coordinates[0]
        y1 = pnt.coordinates[1]
        distance = 0

        pnt_list = []
        dist_list = []

        for pnt2 in points:
            ID2 = pnt2.ID
            x2 = pnt2.coordinates[0]
            y2 = pnt2.coordinates[1]

            if ID1 == ID2:
                pass
            else:
                distance = np.sqrt((x1-x2)**2 + (y1-y2)**2)
                pnt_list.append(ID2)
                dist_list.append(distance)
        
        pnt.storeDist(pnt_list, dist_list)

def main():
    cam = USBcamControl()

    # frame = cam.capture()

    # frame, points = findPoints(frame)

    # calculateDistances(points)
    

    while True:

        frame = cam.capture()

        frame, points = findPoints(frame)

        calculateDistances(points)

        cv2.imshow("frame", frame)

    #To be able to stop the programm
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): 
            break

if __name__ == "__main__":
    main()