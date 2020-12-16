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
        self.frame = cv2.rotate(self.frame, cv2.ROTATE_180_CLOCKWISE)
        return self.frame

def main():
    cam           = USBcamControl()

    while True:
        frame = cam.capture()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #blurred = cv2.GaussianBlur(gray, (11, 11), 0)

        # threshold the image to reveal light regions in the
        # blurred image
        thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

        # perform a connected component analysis on the thresholded
        # image, then initialize a mask to store only the "large"
        # components
        labels = measure.label(thresh, background=0)
        mask = np.zeros(thresh.shape, dtype="uint8")
        # loop over the unique components
        for label in np.unique(labels):
            # if this is the background label, ignore it
            if label == 0:
                continue
            # otherwise, construct the label mask and count the
            # number of pixels 
            labelMask = np.zeros(thresh.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)
            # if the number of pixels in the component is sufficiently
            # large, then add it to our mask of "large blobs"
            if numPixels > 5:
                mask = cv2.add(mask, labelMask)

        # find the contours in the mask, then sort them from left to
        # right
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = contours.sort_contours(cnts)[0]
        # loop over the contours
        for (i, c) in enumerate(cnts):
            # draw the bright spot on the image
            print(f"cnts = {cnts}, I = {i}, C = {c}")

            (x, y, w, h) = cv2.boundingRect(c)
            ((cX, cY), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(frame, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
            cv2.putText(frame, "#{}".format(i + 1), (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        # show the output image

        # perform a series of erosions and dilations to remove
        # any small blobs of noise from the thresholded image
        # thresh = cv2.erode(thresh, None, iterations=2)
        # thresh = cv2.dilate(thresh, None, iterations=4)
        cv2.imshow("frame", frame)

        cv2.imshow("Thresh hold image", mask)

        #To be able to stop the programm
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): 
            break

if __name__ == "__main__":
    main()