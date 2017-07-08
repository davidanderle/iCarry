import time
import numpy as np
import argparse
import imutils
import cv2
import RPi.GPIO as io
from collections import deque
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video import FPS
from imutils.video.pivideostream import PiVideoStream
 
ap = argparse.ArgumentParser()
ap.add_argument("--debug", type = bool, default = False)
args = vars(ap.parse_args())
 
DEBUG = args["debug"]
BUFFER_SIZE = 8
FRAME_RATE = 10
KNOWN_WIDTH = 11.5
FOCAL_LENGHT = 480
FRONT_RIGHT_FORWARD = 3
FRONT_RIGHT_REVERSE = 5
FRONT_LEFT_FORWARD = 8
FRONT_LEFT_REVERSE = 10
REAR_RIGHT_FORWARD = 11
REAR_RIGHT_REVERSE = 13
REAR_LEFT_FORWARD = 16
REAR_LEFT_REVERSE = 18
FREQ = 5000
FRAME_RESOLUTION = (320, 240)
 
def getDistanceToCamera(knownWidth, focalLength, width):
    return (knownWidth*focalLength)/width
 
def getDuty(X):                                     #Arbitrary function for
    Y = 12.1*np.log2(2*abs(X)-10)
    return -Y if np.sign(X) == 1 else Y
 
def distributeDuty(vRel, v, side):
    deltaV = v[0]-v[1]                              #for left turn v1 = left
    y = int((vRel - deltaV)/2)
    v[0] = v[0] + y
    v[1] = v[1] - y
    if v[0] > 100:
        remainder = v[0]%100
        v[0] = v[0] - remainder
        v[1] = v[1] - remainder
    if side == "RIGHT":
        v[0] = -v[0]
        v[1] = -v[1]
    return v
 
def goReverseLeft(duty):
    flf.stop()
    rlf.stop()
    time.sleep(0.000001)
    flr.start(-duty)
    rlr.start(-duty)
def goForwardLeft(duty):
    flr.stop()
    rlr.stop()
    time.sleep(0.000001)
    flf.start(duty)
    rlf.start(duty)
def goReverseRight(duty):
    frf.stop()
    rrf.stop()
    time.sleep(0.000001)
    frr.start(-duty)
    rrr.start(-duty)
def goForwardRight(duty):
    frr.stop()
    rrr.stop()
    time.sleep(0.000001)
    frf.start(duty)
    rrf.start(duty)
 
def getOffset(x):
    if x < 25: return 0
    elif x > 100: return 100
    else: return (130*x - 32.5)
 
io.setmode(io.BOARD)                                    #set RPi pin names by
                                                        #   its numbers
io.setup(FRONT_RIGHT_FORWARD, io.OUT)                   #Assign front right
 
io.setup(FRONT_RIGHT_REVERSE, io.OUT)                   #   motor to pins
frf = io.PWM(FRONT_RIGHT_FORWARD, FREQ)                 #Set PWM frequency to
frr = io.PWM(FRONT_RIGHT_REVERSE, FREQ)                 #   a constant value
 
io.setup(FRONT_LEFT_FORWARD, io.OUT)                    #Assign front left
io.setup(FRONT_LEFT_REVERSE, io.OUT)                    #   motor to pins
flf = io.PWM(FRONT_LEFT_FORWARD, FREQ)
flr = io.PWM(FRONT_LEFT_REVERSE, FREQ)
 
io.setup(REAR_RIGHT_FORWARD, io.OUT)                    #Assign rear right
io.setup(REAR_RIGHT_REVERSE, io.OUT)                    #   motor to pins
rrf = io.PWM(REAR_RIGHT_FORWARD, FREQ)
rrr = io.PWM(REAR_RIGHT_REVERSE, FREQ)
 
io.setup(REAR_LEFT_FORWARD, io.OUT)                     #Assign rear left
io.setup(REAR_LEFT_REVERSE, io.OUT)                     #   motor to pins
rlf = io.PWM(REAR_LEFT_FORWARD, FREQ)
rlr = io.PWM(REAR_LEFT_REVERSE, FREQ)
 
counter = 0                                             #stored frame counter
(X, Y) = (0, 0)                                         #origin of the frame
speed = ""
direction = ""
distance = ""
d = 0
pts = deque(maxlen = BUFFER_SIZE)                       #frame storage size
duty = [0, 0]
 
vs = PiVideoStream().start()                            #start pipelining
time.sleep(2.0)                                         #let camera warm up
 
fps = FPS().start()                                     #start fps counter
 
while True:
    frame = vs.read()                                   #read a frame
    if DEBUG == True:
        frame = imutils.resize(frame,                   #shrink the window of
                width = FRAME_RESOLUTION[1])            #   the video
        status = "No target"
   
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)      #get rid of RGB colors
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)         #blur for cancel high
                                                        #   high freq noise
    edged = cv2.Canny(blurred, 50, 150)                 #detect edges
 
    cnts = cv2.findContours(edged.copy(),               #draw contours
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
 
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01*peri, True)
 
        if len(approx) >= 4 and len(approx) <= 6:       #look for 4|6 vertices
            (x, y, w, h) = cv2.boundingRect(approx)     #sides of a rectangle
 
            aspectRatio = w / float(h)
            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)
 
            keepDims = w > 10 and h > 10                #if sides > 10 px
            keepSolidity = solidity > 0.95              #want vertical edges
            keepAspectRatio = aspectRatio >= 0.5 and aspectRatio <= 2
                                                        #if all is true, it
            if keepDims and keepSolidity and keepAspectRatio:
                if DEBUG == True:                       #   is a square      
                    cv2.drawContours(frame, [approx],   #draw the contours
                        -1, (0, 0, 255), 4)
                    status = "Target acquired"
 
                M = cv2.moments(approx)
                (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
               
                if DEBUG == True:
                    (startX, endX) = (int(cX - (w*0.15)), int(cX + (w*0.15)))
                    (startY, endY) = (int(cY - (h*0.15)), int(cY + (h*0.15)))
               
                    cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
                    cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)
 
                pts.appendleft((cX, cY))
                d = getDistanceToCamera(KNOWN_WIDTH, FOCAL_LENGHT, w)
 
                if DEBUG == True:
                    distance = "Distance = {}".format(d)
   
    for i in range(1, len(pts)):
        if pts[i-1] is None or pts[i] is None:          #if we have 2 frames
            continue                                    #   then OK, else break
 
        if counter >= 10 and i >= 7 and pts[0] is not None:
            X = pts[i][0] - FRAME_RESOLUTION[0]/2       #get X coordinate
            Y = FRAME_RESOLUTION[1]/2 - pts[i][1]       #get Y coordinate
 
            if DEBUG == True: (posX, posY) = ("", "")
 
            if X < -10:                                 #turn right
                if DEBUG == True:
                    print(duty)
 
                dutyCycle = getDuty(-X) + getOffset(d)
                if DEBUG == True:
                    posX = "Turn right"
                    print(dutyCycle)
                duty = distributeDuty(dutyCycle, duty, "RIGHT")
 
                if DEBUG == True:
                    print(duty)
 
                if duty[1] < 0:
                    goReverseLeft(duty[1])
                else:
                    goForwardLeft(duty[1])
                if duty[0] < 0:
                    goReverseRight(duty[0])
                else:
                    goForwardRight(duty[0])
 
            if X > 10:                                  #turn left
                if DEBUG == True:
                    print(duty)
               
                dutyCycle = getDuty(X) + getOffset(d)
 
                if DEBUG == True:
                    print(dutyCycle)
 
                duty = distributeDuty(dutyCycle, duty, "LEFT")
 
                if DEBUG == True:
                    posX = "Turn left"
                    print(duty)
 
                if duty[0] < 0:
                    goReverseLeft(duty[0])
                else:
                    goForwardLeft(duty[0])
                if duty[1] < 0:
                    goReverseRight(duty[1])
                else:
                    goForwardRight(duty[1])
 
            if np.abs(Y) > 5:
                if DEBUG == True:
                    posY = "Slow down" if np.sign(Y) == 1 else "Speed up"
           
            elif np.abs(Y) < 5:
                if DEBUG == True: posY = "Stop"
            if DEBUG == True:
                if (posX != "" or posY != ""):
                    speed = posY
                    direction = posX
 
                elif DEBUG == True:
                    position = ""
 
        if DEBUG == True:
            thickness = int(np.sqrt(BUFFER_SIZE/float(i+1))*2.5)
            cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), thickness)
 
    if DEBUG == True:                                   #Write data on frame
        cv2.putText(frame, distance, (100, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                 (0, 0, 255), 1)
 
        cv2.putText(frame, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                (0, 0, 255), 1)
 
        cv2.putText(frame, speed, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                 (0, 0, 255), 1)
 
        cv2.putText(frame, "(x, y) = ({}, {})".format(X, Y),
                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                (0, 0, 255), 1)
 
        cv2.putText(frame, status, (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                 (0, 0, 255), 1)
 
        cv2.imshow("Edged frame", edged)
        cv2.imshow("Frame", frame)
 
    fps.update()                                        #add 1 to frame counter
    counter += 1                                        #increment loop counter
    key = cv2.waitKey(1) & 0xFF                         #wait for a key order
 
    if key == ord("q"):
        break                                           #end the video loop
 
fps.stop()                                              #stop fps counter
if DEBUG == True:
    print("elapsed time: {:.2f}".format(fps.elapsed())) #print fps
    print("approx FPS: {:.2f}".format(fps.fps()))
 
    cv2.destroyAllWindows()                             #close windows
    vs.stop()                                           #stop video stream
 
frf.stop()                                              #stop PWMs
frr.stop()
flf.stop()
flr.stop()
rrf.stop()
rrr.stop()
rlf.stop()
rlr.stop()
io.cleanup() 
