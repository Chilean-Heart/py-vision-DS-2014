#IMPORTANT!!!
#Use Table "datatable" for testing &
#table "vision" for final robot code.
#DO NOT implement "vision" in simulated robot code!

#Imports
import cv2
import numpy as np
from time import time
from pynetworktables import *

#Debug Flag
debug = False

#Source variables
ret = False
useInternal = True
camClosed = True
internalCam = 1
robotCamIP = "http://10.25.76.11/mjpg/video.mjpg"

#Robot Variables
useLocalRobot = True
robotIP = '10.25.76.2'
localIP = '127.0.0.1'
allianceIsRed = True
#isConnected = False

#Thresholding HSV Values
redMin = np.array([160, 50, 50], np.uint8)
redMax = np.array([180, 200, 200], np.uint8)
blueMin = np.array([70, 50, 50], np.uint8)
blueMax = np.array([135, 200, 200], np.uint8)

#Function Parameters
seSize = (2, 2)        #Element for morphology
se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, seSize)  #Create structuring element before loop
minDist = 100          #Minimum distance between circle center points
radMin = 50            #Minimum detected circle radius
radMax = 250           #Maximum detected circle radius
frameInterval = 33     #Time in ms in between frames
gaussian_power = 5     #Gaussian blur parameter
param1ForCircles = 50  #First Parameter for HoughCircles -
param2ForCircles = 25  #Second Parameter for HoughCircles -
dpForCircles = 2       #DP parameter for HoughCircles -

#Circle Parameters
centerPoint = 0        #Circle center point
radius = 0             #Circle radius
prev_x = 0
prev_y = 0
prev_r = 0

#FPS Calculation
startAtFrame = 10
totalTime = 0
numberOfFrames = 0
time1 = time()
time2 = 0
fps = 0
fpsTextPos = (5, 25)
fpsTextColor = (255, 255, 255)

#Define setting variables
CV_CAP_PROP_FRAME_WIDTH = 3
CV_CAP_PROP_FRAME_HEIGHT = 4

#Window
cv2.namedWindow("Vision DS", 1)

#Extra Images
errorScreen = cv2.imread("system_down.jpg")
blank = np.zeros((400, 400, 3), np.uint8)
canvas = np.zeros((645, 645, 3), np.uint8)
canvas[245:645, :] = (145, 145, 145)

#NetworkTables Client Init
if useLocalRobot:
    NetworkTable.SetIPAddress(localIP)
else:
    NetworkTable.SetIPAddress(robotIP)
NetworkTable.SetClientMode()
NetworkTable.Initialize()

#Table Object
if useLocalRobot:
    table = NetworkTable.GetTable("datatable")
else:
    table = NetworkTable.GetTable("vision")

#Check robot color alliance
def checkAlliance():
    global allianceIsRed
    global isConnected
    try:
        allianceIsRed = table.GetBoolean("allianceRed")
    except TableKeyNotDefinedException:
        isConnected = False

#Initial Robot Connection
def connect():
    global isConnected
    isConnected = False
    while not isConnected:
        cv2.imshow("Vision DS", errorScreen)
        try:
            isConnected = table.GetBoolean("connection")
        except TableKeyNotDefinedException:
            isConnected = False
        checkAlliance()
        if cv2.waitKey(frameInterval) == 27:
            cv2.destroyAllWindows()
            exit(0)

#Callback Function
#def callBack(event, x, y, flags, param):
#    if event == cv2.cv.EVENT_LBUTTONDOWN:
#        print(img[x, y])
#        if debug:
#            print("Click")

#cv2.setMouseCallback("canvas", callBack)

#Choose source for cam
if useInternal:
    cam = cv2.VideoCapture(internalCam)
    cam.set(CV_CAP_PROP_FRAME_WIDTH, 320)
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, 240)
else:
    cam = cv2.VideoCapture(robotCamIP)

#Verify cam
while camClosed:
    if cam.isOpened():
        ret, img = cam.read()
        if ret:
            camClosed = False
        else:
            camClosed = True
    else:
        camClosed = True

#Call connect function
connect()

#Main Loop
while ret:
    if allianceIsRed:
        #Ball Detection for Red
        #Convert & Threshold
        redHSVImage = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        redDetectImage = cv2.inRange(redHSVImage, redMin, redMax)
        #Morphology
        redOpenMorphImage = cv2.morphologyEx(redDetectImage, cv2.MORPH_OPEN, se)
        redCloseMorphImage = cv2.morphologyEx(redOpenMorphImage, cv2.MORPH_CLOSE, se)
        #Gaussian Blur
        redBlurredImage = cv2.GaussianBlur(redCloseMorphImage, (0, 0), gaussian_power)
        #Detect Circles
        circles = cv2.HoughCircles(redBlurredImage, cv2.cv.CV_HOUGH_GRADIENT, dpForCircles,
                                   minDist, param1=param1ForCircles, param2=param2ForCircles,
                                   minRadius=radMin, maxRadius=radMax)
    else:
        #Ball Detection for Blue
        #Convert & Threshold
        blueHSVImage = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blueDetectImage = cv2.inRange(blueHSVImage, blueMin, blueMax)
        #Morphology
        blueOpenMorphImage = cv2.morphologyEx(blueDetectImage, cv2.MORPH_OPEN, se)
        blueCloseMorphImage = cv2.morphologyEx(blueOpenMorphImage, cv2.MORPH_CLOSE, se)
        #Gaussian Blur
        blueBlurredImage = cv2.GaussianBlur(blueCloseMorphImage, (0, 0), gaussian_power)
        #Detect Circles
        circles = cv2.HoughCircles(blueBlurredImage, cv2.cv.CV_HOUGH_GRADIENT, dpForCircles,
                                   minDist, param1=param1ForCircles, param2=param2ForCircles,
                                   minRadius=radMin, maxRadius=radMax)
    if circles != None:
        for i in circles[0, :]:
            centerPoint = (i[0], i[1])
            radius = i[2]
            cv2.circle(img, centerPoint, radius, (0, 255, 0), thickness=2)

    #FPS Calculation
    time2 = time()
    intervalBetweenFrames = time2 - time1
    if numberOfFrames > startAtFrame:
        totalTime += intervalBetweenFrames
        fps = int(round(1 / (totalTime / (numberOfFrames - startAtFrame))))
    time1 = time2
    numberOfFrames += 1
    cv2.putText(img, "FPS: %d" % fps, fpsTextPos, cv2.FONT_HERSHEY_SIMPLEX, 0.65, fpsTextColor, 2)

    #Send data via NetworkTables
    table.PutNumber("FPS", fps)
    if centerPoint != 0 and prev_x != centerPoint[0] and prev_y != centerPoint[1]:
        prev_x = centerPoint[0]
        prev_y = centerPoint[1]
        table.PutNumber("X", int(centerPoint[0]))
        table.PutNumber("Y", int(centerPoint[1]))
        canvas[245:645, :] = (145, 145, 145)
        cv2.putText(canvas, "X Point: %d" % centerPoint[0], (5, 360), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)
        cv2.putText(canvas, "Y Point: %d" % centerPoint[1], (5, 420), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)
    if radius != 0:
        table.PutNumber("R", int(radius))
        cv2.putText(canvas, "Radius: %d" % radius, (5, 480), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)

    #FPS Debug Messages
    if debug:
        print("totalTime: %f" % totalTime)
        print("numberOfFrames: %d" % numberOfFrames)
        print("startAtFrame: %d" % startAtFrame)
        print("intervalBetweenFrames: %f" % intervalBetweenFrames)
        print("FPS: %d" % fps)
        print(" ")

    #Stitch Images
    #Paints borders according to alliance color
    #Show Alliance
    if allianceIsRed:
        canvas[:240, 0:320] = cv2.cvtColor(redBlurredImage, cv2.COLOR_GRAY2RGB)
        canvas[:240, 320:325] = (0, 0, 255)
        canvas[240:245, :] = (0, 0, 255)
        cv2.putText(canvas, "Current Alliance is: ", (5, 300), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)
        canvas[274:309, 360:550] = (0, 0, 255)
        cv2.putText(canvas, "Red", (418, 301), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)

    else:
        canvas[:240, 0:320] = cv2.cvtColor(blueBlurredImage, cv2.COLOR_GRAY2RGB)
        canvas[:240, 320:325] = (255, 0, 0)
        canvas[240:245, :] = (255, 0, 0)
        cv2.putText(canvas, "Current Alliance is: ", (5, 300), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)
        canvas[274:309, 360:550] = (255, 0, 0)
        cv2.putText(canvas, "Blue", (410, 301), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)

    canvas[:240, 325:] = img

    #Display Image
    cv2.imshow("Vision DS", canvas)

    #Read Cam
    ret, img = cam.read()

    #Verify connection
    try:
        isConnected = table.GetBoolean("connection")
    except TableKeyNotDefinedException:
        isConnected = False

    if not isConnected:
        connect()

    #Check alliance every cycle...unnecessary
    #checkAlliance()

    #Delay and exit program
    key = cv2.waitKey(frameInterval)
    if key == 27:
        break

#Cleanly exit program
try:
    table.PutBoolean("connection", False)
except TableKeyNotDefinedException:
    pass
cv2.destroyAllWindows()
exit(0)
