#import RPi.GPIO as GPIO
import pigpio
import time
import math

#mapping values from PID calculation to servo range (1 - 135 degrees)
def mapToServo(PIDoutput:float):
    if PIDoutput != 0:
        PIDoutput += 7000
        if PIDoutput > 14000:
            PIDoutput = 14000
            #print("max")
        if PIDoutput < 1:
            PIDoutput = 1
            #print("min")
        angle = (1375-(1375/(12000/PIDoutput)))+500
        if angle < 500:
            angle = 500
        if angle > 1875:
            angle = 1875
        return angle 
        
#setting given servo to certain value
def setServo(pwm, pin:int, angle:float):
    if not math.isnan(angle):
        pwm.set_servo_pulsewidth(pin, angle)
    else:
        pwm.set_servo_pulsewidth(pin, 500)


#creating new servo at set pin    returning servo obj
def newServo(pin:int, pwm):
    try:
        pwm.set_mode(pin, pigpio.OUTPUT)
        pwm.set_PWM_frequency(pin, 50 )
        return True
    except:
        return False

#destroying given servo
def killServo(pwm, pin:int):
    pwm.set_PWM_dutycycle(pin, 0 )
    pwm.set_PWM_frequency(pin, 0 )
    
