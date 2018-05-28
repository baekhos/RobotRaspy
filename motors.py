#!/usr/bin/env python
#motorp.py

import time
import os
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import sys
#implementam si encoder ul tot aici
#nr de gauri pe encoder este = 20
#diametrul rotii este = 6.4 cm
#pe 20 de roatatii avem 20 de cm
#deci 1 cm pe gaura encoder
#rezolutia encoder ului este de ~0.5 cm

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)
GPIO.setup(40,GPIO.IN)

#setup pwm
fs=GPIO.PWM(11,50) #spate dreapta- ok
fd=GPIO.PWM(15,50) #fata dreapta-ok
ss=GPIO.PWM(7,50) #spate stanga-ok
sd=GPIO.PWM(13,50) #fata dreapta-ok
speed=30
turnspeed=12
def forward():
    
    ss.stop()
    sd.stop()
    fs.start(speed)
    fd.start(speed)
    print "move forward"

def backwards():
    ss.start(speed)
    sd.start(speed)
    fs.stop()
    fd.stop()
    print "move backwards"
def turnRight():
    ss.stop()
    sd.start(turnspeed)
    fs.start(turnspeed)
    fd.stop()
    print "turn right"
def turnLeft():    
    ss.start(turnspeed)
    sd.stop()
    fs.stop()
    fd.start(turnspeed)
    print "turn left"

def EncoderMeasure():
    cm=0;
    oldValue=newValue=GPIO.input(40)    
    end=start= time.time()
    while end - start < 1:
        end= time.time()
        newValue = GPIO.input(40)
        if newValue!=oldValue :
            cm+=0.5
        oldValue=newValue        
    ss.stop()
    sd.stop()
    fs.stop()
    fd.stop()
    return cm

def callback(msg):    
    if msg.data=="W":
        forward()
    elif msg.data == "S":
        backwards()
    elif msg.data == "D":
        turnRight()
    elif msg.data == "A":
        turnLeft()
    elif msg.data == "Q":
        GPIO.cleanup()
        sys.exit()
    cm = EncoderMeasure()
    print cm
    #afiseaza ok citirea de pe encoder pana aici
    #speed is a vlue between 0 and 99 that's what PWM library expects
    #set("duty", str(speed))

             

rospy.init_node('Motor_Subscriber')
sub = rospy.Subscriber('direction', String, callback)
rospy.spin()
GPIO.cleanup()
    
