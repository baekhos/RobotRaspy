import time
import os
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)

def set(property, value):
    try:
        f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
        f.write(value)
        f.close()   
    except:
        print("Error writing to: " + property + " value: " + value)
 
set("delayed", "0")
set("mode", "pwm")
set("frequency", "500")
set("active", "1")

def forward():
    GPIO.output(7,False)
    GPIO.output(11,True)
    GPIO.output(13,False)
    GPIO.output(15,True)
    print "move forward"

def backwards():
    GPIO.output(7,True)
    GPIO.output(11,False)
    GPIO.output(13,True)
    GPIO.output(15,False)
    print "move backwards"
def turnRight():
    GPIO.output(7,True)
    GPIO.output(11,False)
    GPIO.output(13,False)
    GPIO.output(15,True)
    print "turn right"
def turnLeft():
    GPIO.output(7,False)
    GPIO.output(11,True)
    GPIO.output(13,True)
    GPIO.output(15,False)
    print "turn left"

def stop():
    GPIO.output(7,False)
    GPIO.output(11,False)
    GPIO.output(13,False)
    GPIO.output(15,False)

def callback(msg):    
    if msg.data=="W":
        forward()
    elif char == "S":
        backwards()
    elif char == "D":
        turnRight()
    elif char == "A":
        turnLeft()
    time.sleep(0.5)
    stop()
    #speed is a vlue between 0 and 99 that's what PWM library expects
    #set("duty", str(speed))

             

rospy.init_node('Motor_Subscriber')
sub = rospy.Subscriber('direction', String, callback)
rospy.spin()
GPIO.cleanup()
    
