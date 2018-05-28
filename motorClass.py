#!/usr/bin/env python
#motorp.py

import time
import os
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from nav_msgs.msg import Odometry
import RPi.GPIO as GPIO
import sys, math, tf
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

class Motor():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)
        sub = rospy.Subscriber('direction', String, callback)
        
        
        #setup pwm
        self.fs=GPIO.PWM(11,50) #spate dreapta- ok
        self.fd=GPIO.PWM(15,50) #fata dreapta-ok
        self.ss=GPIO.PWM(7,50) #spate stanga-ok
        self.sd=GPIO.PWM(13,50) #fata dreapta-ok
        self.speed=30
        self.turnspeed=12
        self.cm =0
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        #Coordonate carteziene
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def send_odom(self):
        self.cur_time = rospy.Time.now()

        #dt = self.cur_time.to_sec() - self.last_time.to_sec()
        self.x += self.vx * math.cos(self.th) #* dt
        self.y += self.vx * math.sin(self.th) #* dt
        self.th += self.vth #* dt 

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x,self.y,0.0), q, self.cur_time,"base_link","odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position = Point(self.x,self.y,0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)

        self.last_time = self.cur_time

    def forward(self):    
        ss.stop()
        sd.stop()
        fs.start(speed)
        fd.start(speed)
        print "move forward"

    def backwards(self):
        ss.start(speed)
        sd.start(speed)
        fs.stop()
        fd.stop()
        print "move backwards"
    def turnRight(self):
        ss.stop()
        sd.start(turnspeed)
        fs.start(turnspeed)
        fd.stop()
        print "turn right"
    def turnLeft(self):    
        ss.start(turnspeed)
        sd.stop()
        fs.stop()
        fd.start(turnspeed)
        print "turn left"

    def EncoderMeasure(self,command):
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
        if command=="W":
            self.vx+= cm*0.001
        elif command == "S":
            self.vx-=-cm*0.001
        elif command == "D":
            self.vth-=3.14/4
        elif command == "A":
            self.vth+=3.14/4
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
        cm = EncoderMeasure(msg.data)
        print cm

  

if __name__ == '__main__':
    rospy.init_node('Motor_Subscriber')
    m = Motor()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()
    GPIO.cleanup()    