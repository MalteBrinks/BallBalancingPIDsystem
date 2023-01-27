import numpy as np
import cv2

def getColorPositions(frameHSV, range):
    lowerRange, upperRange = range
    mask = cv2.inRange(frameHSV, lowerRange, upperRange)
    allPixels = np.argwhere(mask)
    return allPixels, mask

def averagePixelPos(allPixels):
    x, y=[], []
    for xp, yp in allPixels:
        x.append(xp) 
        y.append(yp)
    average = np.average(x), np.average(y)
    return average