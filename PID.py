from math import sqrt, isnan
#calculating PID function
def calc(input:int, lastInput:int, pid:tuple, integral): #input=wantedDistance-curentDistance
    if not isnan(input):
        wP, wI, wD = pid
        proportional = wP * input
        integral     = (integral + input)*wI
        derivative   = wD * (input - lastInput)
        result = (proportional+integral+derivative)
        return result, integral
    else:
        return 1250, 0
#calculating error = wanted-current
def calcError(wantedDistance, currentDistance):
    input = wantedDistance-currentDistance
    return input

#calculating distance between two coordinates
def calcDistance(pos1:tuple, pos2:tuple):
    x1, y1 = pos1
    x2, y2 = pos2
    distance = sqrt((x1-x2)**2+(y1-y2)**2)
    return distance
    