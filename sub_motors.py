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
        sub = rospy.Subscriber('direction', String, self.callback)
        self.direction="X"
        self.oldValue=self.newValue=GPIO.input(40) 
        #setup pwm
        self.fs=GPIO.PWM(11,50) #spate dreapta- ok
        self.fd=GPIO.PWM(15,50) #fata dreapta-ok
        self.ss=GPIO.PWM(7,50) #spate stanga-ok
        self.sd=GPIO.PWM(13,50) #fata dreapta-ok
        self.speed=30
        self.turnspeed=16
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        #Coordonate carteziene
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def send_odom(self):
        self.cur_time = rospy.Time.now()

        self.EncoderMeasure(self.direction)    
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
        self.ss.stop()
        self.sd.stop()
        self.fs.start(self.speed)
        self.fd.start(self.speed)
        print "move forward"

    def backwards(self):
        self.ss.start(self.speed)
        self.sd.start(self.speed)
        self.fs.stop()
        self.fd.stop()
        print "move backwards"
    def turnRight(self):
        self.ss.stop()
        self.sd.start(self.turnspeed)
        self.fs.start(self.turnspeed)
        self.fd.stop()
        print "turn right"
    def turnLeft(self):    
        self.ss.start(self.turnspeed)
        self.sd.stop()
        self.fs.stop()
        self.fd.start(self.turnspeed)
        print "turn left"
    def stop(self):
        self.ss.stop()
        self.sd.stop()
        self.fs.stop()
        self.fd.stop()
        print "stop"

    def EncoderMeasure(self,direction):
        cm=0
        self.newValue = GPIO.input(40)
        if self.newValue!=self.oldValue :
            cm+=0.5
        self.oldValue=self.newValue
        if cm!=0:             
            if direction=="W":
                self.vx+= cm*0.001
            elif direction == "S":
                self.vx-=cm*0.001
            elif direction == "D":
                self.vth-=3.14*(cm/43.5)
            elif direction == "A":
                self.vth+=3.14*(cm/38)
            
    #43.5 si 38 cm au fost calculate apriori si reprezinta distanta citita
    #de encoder pentru o rotatie de 360 de grade pe roata din dreapta

    def callback(self,msg):
        self.direction=msg.data
        if msg.data=="W":
            self.forward()
        elif msg.data == "S":
            self.backwards()
        elif msg.data == "D":
            self.turnRight()
        elif msg.data == "A":
            self.turnLeft()
        elif msg.data == "X":
            self.stop()
        elif msg.data == "Q":
            GPIO.cleanup()
            ros.shutdown()
        

  

if __name__ == '__main__':
    rospy.init_node('Motor_Subscriber')
    m = Motor()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()
    GPIO.cleanup()    
