import curses
import time, numpy
from adafruit_servokit import ServoKit

stdscr = curses.initscr()

#this stops keyboard input into terminal, but the code still reads it
#curses.noecho()
#this allow each key that is pressed to be input immediately into the code, without having to press enter
#character break
#curses.cbreak()
#special keys equal values
#stdscr.keypad(True)
#Stops blinking cursor on screen
#curses.curs_set(0)


kit = ServoKit(channels=16)




def main(stdscr):
    
    angle0 = 90
    angle1 = 90
    angle3 = 90
    angle8 = 90
    angle10 = 90
    angle11 = 90

    while True:
        stdscr.nodelay(True)
        c = stdscr.getch()
        kit.servo[0].angle = angle0
        #180 is lowest, 40 is highest
        kit.servo[1].angle = angle1
        #180 is high, 40 is lowest
        kit.servo[3].angle = angle3
        #0 is high, 140 is lowest
        kit.servo[8].angle = angle8
        #0 is lowest, 180 is highest
        kit.servo[10].angle = angle10
        #0 is high, 140 lowest
        kit.servo[11].angle = angle11
        #180 is high, 20 is lowest
        if c == curses.KEY_LEFT:
            stdscr.addstr('left\n')
            curses.flushinp()
            angle0 -= 10
            angle1 -= 10
            angle3 += 10
            angle8 += 10
            angle10 -= 10    
            angle11 += 10
            angle0 = numpy.clip(angle0, 0, 140)
            angle1 = numpy.clip(angle1, 40, 180)
            angle3 = numpy.clip(angle3, 0, 140)
            angle8 = numpy.clip(angle8, 40, 180)
            angle10 = numpy.clip(angle10, 0, 140)
            angle11 = numpy.clip(angle11, 40, 180)
        elif c == curses.KEY_RIGHT:
            stdscr.addstr('right\n')
            curses.flushinp()
            angle0 += 10
            angle1 += 10
            angle3 -= 10
            angle8 -= 10
            angle10 += 10    
            angle11 -= 10
            angle0 = numpy.clip(angle0, 0, 140)
            angle1 = numpy.clip(angle1, 40, 180)
            angle3 = numpy.clip(angle3, 0, 140)
            angle8 = numpy.clip(angle8, 40, 180)
            angle10 = numpy.clip(angle10, 0, 140)
            angle11 = numpy.clip(angle11, 40, 180)
        elif c == curses.KEY_UP:
            stdscr.addstr('up\n')
            curses.flushinp()
            angle0 += 10
            angle1 -= 10
            angle3 -= 10
            angle8 += 10
            angle10 -= 10    
            angle11 -= 10
            angle0 = numpy.clip(angle0, 0, 140)
            angle1 = numpy.clip(angle1, 40, 180)
            angle3 = numpy.clip(angle3, 0, 140)
            angle8 = numpy.clip(angle8, 40, 180)
            angle10 = numpy.clip(angle10, 0, 140)
            angle11 = numpy.clip(angle11, 40, 180)
        elif c == curses.KEY_DOWN:
            stdscr.addstr('down\n')
            curses.flushinp()
            angle0 -= 10
            angle1 += 10
            angle3 += 10
            angle8 -= 10
            angle10 += 10    
            angle11 += 10
            angle0 = numpy.clip(angle0, 0, 140)
            angle1 = numpy.clip(angle1, 40, 180)
            angle3 = numpy.clip(angle3, 0, 140)
            angle8 = numpy.clip(angle8, 40, 180)
            angle10 = numpy.clip(angle10, 0, 140)
            angle11 = numpy.clip(angle11, 40, 180)
        elif c == curses.KEY_HOME:
            break
        elif c == curses.KEY_END:
            angle0 = 90
            angle1 = 90
            angle3 = 90
            angle8 = 90
            angle10 = 90    
            angle11 = 90            
        else:
            stdscr.clear()
            stdscr.addstr('waiting\n')
#Prints something on terminal
#stdscr.addstr(5,5,"Hello")
#Refreshes screen with previous commands
#stdscr.refresh()

#changing values back
#curses.echo()
#curses.nocbreak()
#stdscr.keypad(False)
#curses.curs_set(1)

if __name__ == '__main__':
    curses.wrapper(main)
    
    
curses.endwin()