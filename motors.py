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
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)
GPIO.setup(40,GPIO.IN)
GPIO.setup(32,GPIO.IN)

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
        self.impD=self.impS=0
        self.readS=GPIO.input(40)
        self.readD=GPIO.input(32)
        self.distS=0
        self.distD=0        
        #--------------------------------
        self.speed=12
        self.turnspeed=10
        self.limit =25
        self.nt=time.time()
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        #Coordonate carteziene
        self.lastpost=time.time()
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
        self.actual=False
        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def distanceS(self):
        read=GPIO.input(40)
        #print ("encoder stanga : %d" ,read )
        if read==self.readS:
            return 0
        else: 
            self.readS=read
            self.impS+=1
            return 1    
    
    def distanceD(self):
        read=GPIO.input(32)
        #print ("encoder dreapta : %d" ,read )        
        if read==self.readD:
            return 0
        else:
            self.readD=read
            self.impD+=1
            return 1    
    

    def speedCalibration(self):
        self.distD+=self.distanceD()  
        self.distS+=self.distanceS()              
        t=time.time()
        dt=t-self.nt
        if  dt>0.1:            
            speedS=self.distS/dt
            speedD=self.distD/dt            
            print ("viteza pe roata dreapta este: %d iar pwm-ul e de %d iar pe roata stanga este: %d iar pwm-ul e de %d " % ( speedD,self.vd,speedS,self.vs))
            delta=speedS-speedD
            if self.vd < self.limit and self.vs < self.limit:
                if delta> 1.5:                    
                    self.vd+=self.vd*(delta/speedD)
                    if self.vd>self.limit:
                        self.vd=self.limit                
                elif delta < -1.5:
                    
                    delta=abs(delta)
                    self.vs+=self.vs*(delta/speedS)
                    if self.vs>self.limit:
                        self.vs=self.limit
            else:                
                if delta> 1.5:
                    self.vs-=self.vs*(delta/speedS)                    
                elif delta < -1.5:
                    delta=abs(delta)                
                    self.vd-=self.vd*(delta/speedD)
            self.distS=self.distD=0.0001            
            self.nt=time.time()


    def send_odom(self):            
        self.cur_time = rospy.Time.now()
        
         
        currentTime=time.time()
        self.x += self.vx * math.cos(self.th) 
        self.y += self.vx * math.sin(self.th) 
        self.th += self.vth 
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
        self.vx, self.vth = 0.0, 0.0
       
    def forward(self):
        self.fs.start(self.vs)
        self.fd.start(self.vd)  
        self.ss.stop()
        self.sd.stop()
        
    def backwards(self):
        self.ss.start(self.vs)
        self.sd.start(self.vd)        
        self.fs.stop()
        self.fd.stop()
        
    def turnRight(self):        
        self.sd.start(self.vd)
        self.fs.start(self.vs)        
        self.fd.stop()
        self.ss.stop()
        
    def turnLeft(self):    
        self.ss.start(self.vs)
        self.fd.start(self.vd)
        self.fs.stop()
        self.sd.stop()
        
    def stop(self):
        self.ss.stop()
        self.sd.stop()
        self.fs.stop()
        self.fd.stop()
           

    def CurrentPosition(self):          
        if self.directive=="W":            
            self.vx+= 0.15
        elif self.directive == "S":            
            self.vx-= 0.15
        elif self.directive == "D":
            self.vth-=0.7852
        elif self.directive == "A":
            self.vth+=0.7852
        self.directive="X"
        self.impD=self.impS=0
        self.stop()
    #43.5 si 38 cm au fost calculate apriori si reprezinta distanta citita
    #de encoder pentru o rotatie de 360 de grade pe roata din dreapta

    def motorFunction(self):        
        d=(self.impD+self.impS)/2
        print d
        if self.directive=="W" and d<30:
            self.forward()
        elif self.directive == "S" and d<30:
            self.backwards()        
        elif self.directive == "D"and d<19:
            self.turnRight()
        elif self.directive == "A"and d<19:
            self.turnLeft()             
        else:
            self.CurrentPosition()

    def callback(self,msg):        
        self.directive=msg.data
        if self.directive =="W" or self.directive=="S":
            self.vs=16
            self.vd=19
            self.limit =25
        elif self.directive =="A" or self.directive =="D":
            self.vs=18
            self.vd=21
            #se aplica o corectie initiala de 3 pentru motorul din dreapta care e mai rapid
            self.limit=22
        self.nt=time.time()
        distD=distS=0

  

if __name__ == '__main__':
    rospy.init_node('Motor_Subscriber')
    m = Motor()

    rate = rospy.Rate(300)
    while not rospy.is_shutdown():
        if m.directive == "Q":
            GPIO.cleanup()
            ros.shutdown()
        currentTime=time.time()        
        if currentTime-m.lastpost >0.1:
            m.send_odom()
            m.lastpost=currentTime
        if m.directive !="X":    
            m.motorFunction()
            m.speedCalibration()
        rate.sleep()
    GPIO.cleanup()    
