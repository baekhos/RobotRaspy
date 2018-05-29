# import curses and GPIO
import curses
import RPi.GPIO as GPIO


#set GPIO numbering mode and define output pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)
GPIO.setup(40,GPIO.IN)
cm=0;
oldValue=newValue=GPIO.input(40)    
    
# Get the curses window, turn off echoing of keyboard to screen, turn on
# instant (no waiting) key response, and use special values for cursor keys
screen = curses.initscr()
curses.noecho() 
curses.cbreak()
screen.keypad(True)

#setup pwm
fs=GPIO.PWM(11,50) #spate dreapta- ok
fd=GPIO.PWM(15,50) #fata dreapta-ok
ss=GPIO.PWM(7,50) #spate stanga-ok
sd=GPIO.PWM(13,50) #fata dreapta-ok
speed=30
turnspeed=12

def EncoderMeasure():
    end=start= time.time()
    while end - start < 1:
        end= time.time()
        newValue = GPIO.input(40)
        if newValue!=oldValue :
            cm+=0.5
        oldValue=newValue        
    
    

try:
        while True:   
            char = screen.getch()
            if char == ord('q'):
                break
            elif char == curses.KEY_UP:
                ss.stop()
                sd.stop()
                fs.start(speed)
                fd.start(speed)
            elif char == curses.KEY_DOWN:
                ss.start(speed)
                sd.start(speed)
                fs.stop()
                fd.stop()
            elif char == curses.KEY_RIGHT:
                ss.stop()
                sd.start(turnspeed)
                fs.start(turnspeed)
                fd.stop()
            elif char == curses.KEY_LEFT:
                ss.start(turnspeed)
                sd.stop()
                fs.stop()
                fd.start(turnspeed)
            elif char == 10:
                print cm
                cm=0
                ss.stop()
                sd.stop()
                fs.stop()
                fd.stop()
             
finally:
    #Close down curses properly, inc turn echo back on!
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
    GPIO.cleanup()
