from picamera.array import PiRGBArray
from picamera import PiCamera

#importing librarys
import cv2
import numpy as np
import math
from time import sleep, time

#importing functions from scripts
from colorDetection import *
from PID import *
from servo import *
"""
green = Motor A
blue = MOtor B
yellow = Motor C
"""
redRange = np.array([6,81,130]), np.array([41,255,233])

pid = 11, .96, 204   #15, .96, 204         11, .96, 204 
greenPIDval = pid
bluePIDval = pid
yellowPIDval = pid

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

greenPos, bluePos, yellowPos = (495, 464), (97, 258), (469, 47)
greenPin, bluePin, yellowPin = 17, 27, 22
PWM = pigpio.pi()

for pin in [greenPin, bluePin, yellowPin]:
    newServo(pin, PWM)

greenError = blueError = yellowError = greenIntegral = blueIntegral = yellowIntegral = 0

wantedDisGreen = calcDistance(greenPos, (300,300))
wantedDisBlue = calcDistance(bluePos, (300,300))
wantedDisYellow = calcDistance(yellowPos, (300,300))

start = time.time()
count = 0
sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    try:
        count += 1
        frame = frame.array
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        ballPos, mask = getColorPositions(frameHSV, redRange)
        ballY, ballX = averagePixelPos(ballPos)
        
        if not (math.isnan(ballX) or math.isnan(ballY)):
            greenDistance = calcDistance((ballX, ballY), greenPos)
            blueDistance = calcDistance((ballX, ballY), bluePos)
            yellowDistance = calcDistance((ballX, ballY), yellowPos)
            
            oldGreenError = greenError
            oldBlueError = blueError
            oldYellowError = yellowError
            
            greenError = calcError(wantedDisGreen, greenDistance)
            blueError = calcError(wantedDisBlue, blueDistance)
            yellowError = calcError(wantedDisYellow, yellowDistance)
            
            print(greenError, blueError, yellowError)

            greenPID, greenIntegral = calc(greenError, oldGreenError, (greenPIDval), greenIntegral)
            bluePID, blueIntegral =  calc(blueError, oldBlueError, (bluePIDval), blueIntegral)
            yellowPID, yellowIntegral = calc(yellowError, oldYellowError, (yellowPIDval), yellowIntegral)
            
            greenRotation = mapToServo(greenPID)
            blueRotation = mapToServo(bluePID)
            yellowRotation = mapToServo(yellowPID)
            
            setServo(PWM, greenPin, greenRotation)
            setServo(PWM, bluePin, blueRotation)
            setServo(PWM, yellowPin, yellowRotation)
        else:
            
            setServo(PWM, greenPin, 500)
            setServo(PWM, bluePin, 2500)
            setServo(PWM, yellowPin, 500)

        
        cv2.circle(frame, greenPos, int(wantedDisGreen), (255,0,0))
        cv2.circle(frame, bluePos, int(wantedDisBlue), (255,0,0))
        cv2.circle(frame, yellowPos, int(wantedDisYellow), (255,0,0))
        cv2.circle(frame, greenPos, 10, (255,0,0))
        cv2.circle(frame, bluePos, 10, (255,0,0))
        cv2.circle(frame, yellowPos, 10, (255,0,0))
        
        #cv2.imshow("mask", mask)
        #cv2.imshow("frame", frame)
        #cv2.imwrite("/home/malte/frame.jpeg", frame)

        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        if key == ord("q"):
            break
    except:
        break
killServo(PWM, greenPin)
killServo(PWM, bluePin)
killServo(PWM, yellowPin)

cv2.destroyAllWindows()

end = time.time()
print("das Programm hat", end-start, "Sekunden gedauert!")
print("die position wurde ", count, "mal Ã¼berprÃ¼ft!")
