import rospy

from std_msgs.msg import String

# screen = curses.initscr()
# curses.noecho() 
# curses.cbreak()
# screen.keypad(True) 

rospy.init_node('Motor_Publisher')	# Initializing the node
pub = rospy.Publisher('direction', String, queue_size=10)

while not rospy.is_shutdown():		# The function will return True if the node is ready to be shut down and False otherwise
    # This node publishes the input data on the topic "rotation"
    rate = rospy.Rate(1)
    data = raw_input('Enter W/A/S/D to controlthe robot')
    #data = screen.getch()
    data = data.upper()
    if data in ("W","S","A","D" ):
        pub.publish(data)
        rate.sleep()