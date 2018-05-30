#!/usr/bin/env python
#motorp.py

import rospy
import curses
from std_msgs.msg import String
import time
screen = curses.initscr()
curses.noecho() 
curses.cbreak()
screen.keypad(True) 

rospy.init_node('Motor_Publisher')	# Initializing the node
pub = rospy.Publisher('direction', String, queue_size=10)
print 'Enter W/A/S/D to controlthe robot: '
while not rospy.is_shutdown():		# The function will return True if the node is ready to be shut down and False otherwise
    # This node publishes the input data on the topic "rotation"
    rate = rospy.Rate(1)
    print 'Input:'
    try:
        data = chr(screen.getch())
    except:
        data= 'y'
        print('Invalid input')
    data = data.upper()
    print data       
    if data in ("W","S","A","D","Q"):
        pub.publish(data)
        rate.sleep()
    time.sleep(1)
