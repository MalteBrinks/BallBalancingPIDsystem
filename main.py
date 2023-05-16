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

pid = (11, .96, 204)
greenPIDval = bluePIDval = yellowPIDval = pid

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
            positions = [greenPos, bluePos, yellowPos]
            distances = []
            for pos in positions:
                distance = calcDistance((ballX, ballY), pos)
                distances.append(distance)

            greenDistance, blueDistance, yellowDistance = distances
            
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
            
            for pin in [greenPin, bluePin, yellowPin]:
                setServo(PWM, pin, greenRotation)
        else:
            
            setServo(PWM, greenPin, 500)
            setServo(PWM, bluePin, 2500)
            setServo(PWM, yellowPin, 500)

        for pos in [greenPos, bluePos, yellowPos]:
            cv2.circle(frame, pos, int(wantedDisGreen), (255,0,0))
            cv2.circle(frame, pos, 10, (255,0,0))

        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        if key == ord("q"):
            break
    except:
        break

for pin in [greenPin, bluePin, yellowPin]:
    killServo(PWM, pin)

cv2.destroyAllWindows()

end = time.time()
print("das Programm hat", end-start, "Sekunden gedauert!")
print("die position wurde ", count, "mal Ã¼berprÃ¼ft!")
