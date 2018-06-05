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
GPIO.setup(38,GPIO.IN)

class Motor():
    def __init__(self):        
        sub = rospy.Subscriber('direction', String, self.callback)
        
         
        #setup pwm
        self.fs=GPIO.PWM(11,50) #spate dreapta- ok
        self.fd=GPIO.PWM(15,50) #fata dreapta-ok
        self.ss=GPIO.PWM(7,50) #spate stanga-ok
        self.sd=GPIO.PWM(13,50) #fata dreapta-ok
        #-----------------------------------
        self.directive='X'
        #vom declara 2 viteze ale rotilor
        #Viteza rotii stangi si viteza rotii drepte 
        self.vs=self.vd=0
        #--------------------------------------------
        #de asemenea vom avea nevoie de numarul de rotatii pe fiecare roata
        #si de citirea anterioara a encoder ului pe fiecare roata
        self.readS=GPIO.input(40)
        self.readD=GPIO.input(38)
        self.distS=0
        self.distD=0
        #--------------------------------
        self.speed=30
        self.turnspeed=16
        self.nt=0
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        #Coordonate carteziene
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def distanceS(self):
        read=GPIO.input(40)
        if read==self.readS:
            return 0
        elif 
            readS=read
            return 1    
    
    def distanceD(self):
        read=GPIO.input(38)
        if read==self.readD:
            return 0
        elif 
            readD=read
            return 1    
    

    def speedCalibration(self):
        self.distS+=distanceS()
        self.distD+=distanceD()        
        t=rospy.Time.now()
        dt=t-self.nt
        if  dt>0.5:
            speedS=self.distS/dt
            speedD=self.distD/dt
            delta=speedS-speedD
            if delta > 0:
                self.vd+=self.vd*(delta/speedD)
            elif delta <0:
                delta=-delta
                self.vs+=self.vs*(delta/speedS)
            self.distS=self.distD=0
            self.nt=0


    def send_odom(self):
        self.cur_time = rospy.Time.now()

        EncoderMeasure()   
        
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

        motorFunction()
        speedCalibration()

        self.last_time = self.cur_time

    def forward(self):    
        self.ss.stop()
        self.sd.stop()
        self.fs.start(self.vs)
        self.fd.start(self.vd)
        print "move forward"

    def backwards(self):
        self.ss.start(self.vs)
        self.sd.start(self.vd)
        self.fs.stop()
        self.fd.stop()
        print "move backwards"
        
    def turnRight(self):
        self.ss.stop()
        self.sd.start(self.vd)
        self.fs.start(self.vs)
        self.fd.stop()
        print "turn right"
        
    def turnLeft(self):    
        self.ss.start(self.vs)
        self.sd.stop()
        self.fs.stop()
        self.fd.start(self.vd)
        print "turn left"
        
    def stop(self):
        self.ss.stop()
        self.sd.stop()
        self.fs.stop()
        self.fd.stop()
        print "stop"   

    def EncoderMeasure(self):
        step=False
        newValue = GPIO.input(40)
        if newValue!=self.oldValue :
            step=True
        self.oldValue=newValue
        if step == True:             
            if self.directive=="W":
                self.vx+= 0.05
            elif self.directive == "S":
                self.vx-= 0.05
            elif self.directive == "D":
                self.vth-=0,0357
            elif self.directive == "A":
                self.vth+=0,0412
            
    #43.5 si 38 cm au fost calculate apriori si reprezinta distanta citita
    #de encoder pentru o rotatie de 360 de grade pe roata din dreapta

    def motorFunction(self):        
        if self.directive=="W":
            self.forward()
        elif self.directive == "S":
            self.backwards()
        elif self.directive == "D":
            self.turnRight()
        elif self.directive == "A":
            self.turnLeft()
        elif self.directive == "X":
            self.stop()
        elif self.directive == "Q":
            GPIO.cleanup()
            ros.shutdown()

    def callback(self,msg):        
        self.directive=msg.data
        if self.directive =="W" or self.directive=="S":
            self.vs=self.vd=30
        elif self.directive =="A" or self.directive =="D":
            self.vs=self.vd=16
        self.nt=rospy.Time.now()
        distD=distS=0

  

if __name__ == '__main__':
    rospy.init_node('Motor_Subscriber')
    m = Motor()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()
    GPIO.cleanup()    
