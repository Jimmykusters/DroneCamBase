#This file was created to test the can detection
##Code is implemented in robot.py
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import numpy as np
import time
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray

greenColor = np.uint8([[[0,200,0]]]) 
redColor = np.uint8([[[0,0,255]]]) 
WhiteColor = np.uint8([[[230,230,230]]]) 

sampleRate = 10
sampleCounter = 0

#camSetting
resolution = (960, 720)
rotation = 180
framerate = 15

term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 300, 1)
   
##INIT windows location
x,y,width,height = 250,90,400,125


class DetectCans():
    def __init__(self, color, frame):
        self.channel = 0
        self.determineHue(color)
        self.determinRange()

    def determineHue(self, color, frame):
        self.hsv_color = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.hue = self.hsv_color[0][0][0]

    def determinRange(self):
            lower_red = np.array([30,150,50])
            upper_red = np.array([255,255,180])
    
            self.mask = cv2.inRange(self.hsv_color, lower_red, upper_red)
            
        # if self.hue <= 30: # red
        #     self.lower_range = np.array([0, 70, 50], dtype=np.uint8)
        #     self.upper_range = np.array([10, 255, 255], dtype=np.uint8)

        #     self.lower_range2 = np.array([170, 70, 50], dtype=np.uint8)
        #     self.upper_range2 = np.array([180, 255, 255], dtype=np.uint8)

        #     self.mask1      = cv2.inRange(self.hsv_color, self.lower_range, self.upper_range)
        #     self.mask2      = cv2.inRange(self.hsv_color, self.lower_range2, self.upper_range2)
        #     self.mask       = cv2.bitwise_or(self.mask1, self.mask2)
        #     self.roi_hist  = cv2.calcHist([self.hsv_color],[self.channel],self.mask,[180],[0,180])
        #     cv2.normalize(self.roi_hist,self.roi_hist,0,255,cv2.NORM_MINMAX)
        # else: #greem
        #     self.lower_range = np.array([self.hue-15, 100, 50], dtype=np.uint8)
        #     self.upper_range = np.array([self.hue+15, 255, 255], dtype=np.uint8)
        #     self.mask = cv2.inRange(self.hsv_color, self.lower_range, self.upper_range)
        #     self.roi_hist = cv2.calcHist([self.hsv_color],[self.channel],self.mask,[180],[0,180])
        #     cv2.normalize(self.roi_hist,self.roi_hist,0,255,cv2.NORM_MINMAX)

    def loopDetection(self, frame):
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.res = cv2.bitwise_and(frame, frame, mask= self.mask)
        # self.dst = cv2.calcBackProject([self.hsv],[self.channel], self.roi_hist,[0,180],1)
        # self.ret, self.track_window = cv2.CamShift(self.dst, (x,y,width,height), term_crit)

class drawFrameElements():
    def __init__(self):
        #text settings
        self.fontFace   = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale  = 1
        #Colors
        self.green = (0,255,0)
        self.red   = (0,0,255)
        #positions
        self.greenPos   = (40,40)
        self.redPos     = (40,80)

    def drawGreenText(self, frame, text):
         cv2.putText(frame, text, self.greenPos, self.fontFace, self.fontScale, self.green)

    def drawRedText(self, frame, text):
         cv2.putText(frame, text, self.redPos, self.fontFace, self.fontScale, self.red)

    def getPoint(self, ret):
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)
        return(pts)

    def drawGreenBox(self, frame, ret):
        pts = self.getPoint(ret)
        self.drawGreenText(frame, str(pts))
        box = cv2.polylines(frame,[pts],True,self.green,10)

    def drawRedBox(self, frame, ret):
        pts = self.getPoint(ret)
        self.drawRedText(frame, str(pts))
        box = cv2.polylines(frame,[pts],True,self.red,10)

    def drawDetectionbox(self, frame):
        pts = np.array([[140,140],[500,140],[500,340],[140,340]], np.int32)
        cv2.polylines(frame,[pts],True,(0,255,255))

    def showFrame(self, frame, title):
        cv2.imshow(title, frame)

class camControl():
    def __init__(self):
        self.cam = PiCamera()
        self.cam.resolution = resolution
        self.cam.framerate = framerate
        self.cam.rotation = rotation
        self.cam.exposure_mode = 'night'

        self.rawCap = PiRGBArray(self.cam)
        time.sleep(0.1)

    def capture(self):
        self.cam.capture(self.rawCap, format="bgr")
        self.frame = np.array(self.rawCap.array)
        return self.frame

    def clear(self):
        self.rawCap.truncate(0)

class USBcamControl():
    def __init__(self):
        self.cap = cv2.VideoCapture(-1)

    def capture(self):
        self.ret, self.raw = self.cap.read()
        self.frame = np.array(self.raw)
        return self.frame

    # def clear(self):
    #     self.cap.truncate(0)

# def main():
# 	# initialize the camera and grab a reference to the raw camera capture
# 	camera = camControl()
# 	rawCapture = PiRGBArray(camera, size=(640, 480))
# 	# allow the camera to warmup
# 	time.sleep(0.1)
# 	# capture frames from the camera
# 	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
# 		# grab the raw NumPy array representing the image, then initialize the timestamp
# 		# and occupied/unoccupied text
# 		image = frame.array
# 		# show the frame
# 		cv2.imshow("Frame", image)
# 		key = cv2.waitKey(1) & 0xFF
# 		# clear the stream in preparation for the next frame
# 		rawCapture.truncate(0)
# 		# if the `q` key was pressed, break from the loop
# 		if key == ord("q"):
# 			break

# # stap 1:
def main():
    cam           = USBcamControl()
    frameElements = drawFrameElements()

    first_frame = cam.capture()
    frameElements.drawDetectionbox(first_frame)
    #White = DetectCans(WhiteColor, first_frame)
    red   = DetectCans(redColor, first_frame)

    #cam.clear()

    while True:
        frame = cam.capture()

        red.loopDetection(frame)
        # green.loopDetection(frame)

        frameElements.drawRedBox(frame, red.ret)
        # frameElements.drawGreenBox(frame, green.ret)
        # # frameElements.drawDetectionbox(frame)
        frameElements.showFrame(frame, "Test")
        frameElements.showFrame(red.res, "hsv")
        # frameElements.showFrame(green.dst, "dst")
        #cam.clear()

        #To be able to stop the programm
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): 
            break

if __name__ == "__main__":
    main()