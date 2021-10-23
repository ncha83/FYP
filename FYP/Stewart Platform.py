import smbus2
import time, curses, threading
import math as m
import numpy as np
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
from adafruit_servokit import ServoKit
from scipy.optimize import curve_fit
#Initialise curses module
stdscr = curses.initscr()

kit = ServoKit(channels=16)
bus = smbus2.SMBus(1)

#Registers 
IMU_ADD = 0x68
PWMBOARD_ADD = 0x40
##Used to reset sensors 
PWR_MGMT_1 = 0x6B
##Used to control output rate of sensor
SMPLRT_DIV = 0x19
##Used mainly for config of FIFO 
CONFIG =  0x1A
##Config for Gyro 
GYRO_CONFIG = 0x1B
##Config for Accel 
ACCEL_CONFIG = 0x1C
##Config for interrupt pins 
INT_PIN_CFG = 0x37
##Interrupt enable 
INT_ENABLE = 0x38
##Accel and gyro registers 
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48


def IMU_start():
    #Resetting IMU - resets to 0
    bus.write_byte_data(IMU_ADD, PWR_MGMT_1, 0x80)
    time.sleep(0.1)
    #Clock settings - using PLL if possible
    bus.write_byte_data(IMU_ADD, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    #Config to change rate at which sensor outputs values - see below
    sdiv = 0x00 #Set the demoninator: SAMPLE RATE = 8kHz/(1+sdiv)
    bus.write_byte_data(IMU_ADD, SMPLRT_DIV, sdiv)
    time.sleep(0.1)
    #General config - to enable FIFO if required
    bus.write_byte_data(IMU_ADD, CONFIG, 0x00)
    time.sleep(0.1)
    #Gyro and Accel configs
    gconfig = [0x00, 0x08, 0x10, 0x18]
    gvals = [250, 500, 1000, 2000]
    gselect = 0
    
    aconfig = [0x00, 0x08, 0x10, 0x18]
    avals = [2, 4, 8, 16]
    aselect = 0
    
    #Write data to registers
    bus.write_byte_data(IMU_ADD, GYRO_CONFIG, gconfig[gselect])
    time.sleep(0.1)
    bus.write_byte_data(IMU_ADD, ACCEL_CONFIG, aconfig[aselect])
    time.sleep(0.1)
    
    return gvals[gselect], avals[aselect]
   
#Function to retrieve sensor values and convert to signed values
def sensval(register):
    #Get sensor values
    h = bus.read_byte_data(IMU_ADD, register)
    l = bus.read_byte_data(IMU_ADD, register+1)
    
    value = ((h << 8) | l)
    
    #Converting to signed value
    if (value > 32768):
        value -= 65536
    
    return value

#Converting sensor counts to unit values
def sensout(gconfig, aconfig):
    #Gyro output
    gy_x = sensval(GYRO_XOUT_H)
    gy_y = sensval(GYRO_YOUT_H)
    gy_z = sensval(GYRO_ZOUT_H)
    
    #Accel output
    ac_x = sensval(ACCEL_XOUT_H)
    ac_y = sensval(ACCEL_YOUT_H)
    ac_z = sensval(ACCEL_ZOUT_H)
    
    #Convert outputs to proper units (g and dps)
    g_x = (gy_x/(2**15))*gconfig
    g_y = (gy_y/(2**15))*gconfig
    g_z = (gy_z/(2**15))*gconfig
    
    a_x = (ac_x/(2**15))*aconfig
    a_y = (ac_y/(2**15))*aconfig
    a_z = (ac_z/(2**15))*aconfig
    
    return g_x, g_y, g_z, a_x, a_y, a_z

#Function to reset platform to flat state
def resetplat(s0, s2, s3, s8, s10, s11):
    kit = ServoKit(channels=16)
    kit.servo[0].angle = s0
    kit.servo[1].angle = s2
    kit.servo[3].angle = s3
    kit.servo[8].angle = s8
    kit.servo[10].angle = s10
    kit.servo[11].angle = s11
    
    return

def fiteqn(x_input, m_x, b):
    return (m_x*x_input)+b

def fiteqn2(x_input, a0, a1, a3, a8, a10, a11):
    a,b,c,d,e,f,g = x_input
    return a*a0 + b*a1 + c*a3 + d*a8 + e*a10 + f*a11 + g

def autocalib(Ax, Ay, Az, GyroX, GyroY, GyroZ):
    
    roll = m.degrees(m.atan(Ay/m.sqrt(pow(Ax,2) + pow(Az,2))))
    pitch = m.degrees(m.atan(Ax/m.sqrt(pow(Ay,2) + pow(Az,2))))
    GyroX = 0
    GyroY = 0
    GyroZ = 0
    
    return roll, pitch, GyroX, GyroY, GyroZ

def oriencalc(Gx, Gy, Gz, gyroX, gyroY, gyroZ, Ax, Ay, Az, timen, steady):
    pasttime = timen        
    timen = time.time()
    timepassed = timen - pasttime

        
    pitchAcc = m.degrees(m.atan(Ax/m.sqrt(pow(Ay,2) + pow(Az,2))))
    rollAcc = m.degrees(m.atan(Ay/m.sqrt(pow(Ax,2) + pow(Az,2))))
     
    #Integrating gyro value to get an angle
    gyroX = gyroX + Gx*timepassed
    gyroY = gyroY + Gy*timepassed
    gyroZ = gyroZ + Gz*timepassed
    
    #passing the gyro through high pass filter and accel through a low pass filter
    if steady == 1:
        roll = 0*gyroX + rollAcc
        pitch = 0*gyroY + pitchAcc
        yaw = gyroZ
    elif steady == 0:    
        roll = 0.98*gyroX + 0.02*rollAcc
        pitch = 0.98*gyroY + 0.02*pitchAcc
        yaw = gyroZ
        
    return roll, pitch, yaw, gyroX, gyroY, gyroZ, timen

def compare(calcroll, droll, calcpitch, dpitch, breakflag):
    check = 0
#     if ((droll > 0 and calcroll > 0) or (droll < 0 and calcroll < 0)) and (droll - calcroll < 0.5 and droll - calcroll > -0.5):
#         if ((dpitch > 0 and calcpitch > 0) or (dpitch < 0 and calcpitch < 0)) and (dpitch - calcpitch < 0.5 and dpitch - calcpitch > -0.5):
#             check = 1
#             breakflag = 1
#         elif ((dpitch > 0 and calcpitch < 0) or (dpitch < 0 and calcpitch > 0)) and (dpitch + calcpitch < 0.5 and dpitch + calcpitch > -0.5):
#             check = 2
#             breakflag = 1
#     elif ((droll > 0 and calcroll < 0) or (droll < 0 and calcroll > 0)) and (droll + calcroll < 0.5 and droll + calcroll > -0.5):
#         if ((dpitch > 0 and calcpitch > 0) or (dpitch < 0 and calcpitch < 0)) and (dpitch - calcpitch < 0.5 and dpitch - calcpitch > -0.5):
#             check = 3
#             breakflag = 1
#         elif ((dpitch > 0 and calcpitch < 0) or (dpitch < 0 and calcpitch > 0)) and (dpitch + calcpitch < 0.5 and dpitch + calcpitch > -0.5):
#             check = 4
#             breakflag = 1
    
    if (droll > 0 and calcroll > 0) and (droll - calcroll < 0.25 and droll - calcroll > -0.25):
        if (dpitch > 0 and calcpitch > 0)  and (dpitch - calcpitch < 0.25 and dpitch - calcpitch > -0.25):
            check = 1
            breakflag = 1
        elif (dpitch < 0 and calcpitch < 0) and (dpitch - calcpitch < 0.25 and dpitch - calcpitch > -0.25):
            check = 2
            breakflag = 1
    elif (droll < 0 and calcroll < 0)  and (droll - calcroll < 0.25 and droll - calcroll > -0.25):
        if (dpitch > 0 and calcpitch > 0)  and (dpitch - calcpitch < 0.25 and dpitch - calcpitch > -0.25):
            check = 3
            breakflag = 1
        elif (dpitch < 0 and calcpitch < 0) and (dpitch - calcpitch < 0.25 and dpitch - calcpitch > -0.25):
            check = 4
            breakflag = 1
            
    return breakflag, check

def setorien(droll, dpitch):
    #General model offsets
    
    roff = [-0.02288286, -0.01019273,  0.01084114, -0.01016735,  0.01275605, 0.0210254 , -0.15827488] 
    poff = [ 1.41751055e-03,  1.72589736e-02, -2.11304719e-02, -1.81380203e-02, 1.78206054e-02, -3.93356988e-04, -2.76518898e+00]
    yoff = [ 8.06971406e-03, -1.40381726e-03, -2.19322642e-02,  1.13829698e-01, -5.07213721e-01, -4.24976157e-01,  1.40823415e+02]

    a = [80, 90, 90, 90, 115, 95]
    
    breakflag = 0
    calcroll = 0
    calcpitch = 0
    comp = 0
    
    for i in range (0,5):
        comp, check = compare(calcroll, droll, calcpitch, dpitch, breakflag)
        if comp == 1 or breakflag == 1:
            breakflag = 1
            break
        check2 = 1
        
        a[0] = i*45        
        calcroll = fiteqn2(roff, a[0], a[1], a[2], a[3], a[4], a[5])
        calcpitch = fiteqn2(poff, a[0], a[1], a[2], a[3], a[4], a[5])
        
        for ii in range (0,5):
            comp, check = compare(calcroll, droll, calcpitch, dpitch, breakflag)
            if comp == 1 or breakflag == 1:
                breakflag = 1              
                break
            
            check2 = 2  
            a[1] = ii*45       
            calcroll = fiteqn2(roff, a[0], a[1], a[2], a[3], a[4], a[5])
            calcpitch = fiteqn2(poff, a[0], a[1], a[2], a[3], a[4], a[5])
            
            for iii in range (0,5):
                comp, check = compare(calcroll, droll, calcpitch, dpitch, breakflag)
                if comp == 1 or breakflag == 1:
                    breakflag = 1
                    break
                
                check2 = 3
                a[2] = iii*45       
                calcroll = fiteqn2(roff, a[0], a[1], a[2], a[3], a[4], a[5])
                calcpitch = fiteqn2(poff, a[0], a[1], a[2], a[3], a[4], a[5])
                
                for iiii in range (0,5):
                    comp, check = compare(calcroll, droll, calcpitch, dpitch, breakflag)
                    if comp == 1 or breakflag == 1:
                        breakflag = 1
                        break
                    
                    check2 = 4
                    a[3] = iiii*45       
                    calcroll = fiteqn2(roff, a[0], a[1], a[2], a[3], a[4], a[5])
                    calcpitch = fiteqn2(poff, a[0], a[1], a[2], a[3], a[4], a[5])
                    
                    for iiiii in range (0,5):
                        comp, check = compare(calcroll, droll, calcpitch, dpitch, breakflag)
                        if comp == 1 or breakflag == 1:
                            breakflag = 1
                            break
                        
                        check2 = 5
                        a[4] = iiiii*45      
                        calcroll = fiteqn2(roff, a[0], a[1], a[2], a[3], a[4], a[5])
                        calcpitch = fiteqn2(poff, a[0], a[1], a[2], a[3], a[4], a[5])
                        
                        for iiiiii in range (0,5):
                            comp, check = compare(calcroll, droll, calcpitch, dpitch, breakflag)
                            if comp == 1 or breakflag == 1:
                                breakflag = 1
                                break
                            
                            check2 = 6
                            a[5] = iiiiii*45       
                            calcroll = fiteqn2(roff, a[0], a[1], a[2], a[3], a[4], a[5])
                            calcpitch = fiteqn2(poff, a[0], a[1], a[2], a[3], a[4], a[5])
                            
                            print("\n\rCount: ", iiiiii)
                            stdscr.clear()
                            stdscr.refresh()
                            
        
    return a, calcroll, droll, calcpitch, dpitch

def setplat(a0, a1, a3, a8, a10, a11):
    kit.servo[0].angle = a0
    kit.servo[1].angle = a1
    kit.servo[3].angle = a3
    kit.servo[8].angle = a8
    kit.servo[10].angle = a10
    kit.servo[11].angle = a11
    
    return

#Main code
def main(stdscr):
    time.sleep(0.1)
    timen = time.time()
    #Initialize platform 
    out = 90*np.ones(6)
    resetplat(out[0], out[1], out[2], out[3], out[4], out[5])

#   Data used to find the offsets due to error in initial angle
    avgcalc = [[],[],[],[],[],[]]
    avgoffset = [[],[],[],[],[],[]]
    avgcount = 0
    out1 = 0
    out2 = 0
    out3 = 0
    out4 = 0
    pasttime = 0
    #Get gyro and accel config vals
    gconfig, aconfig = IMU_start()
    time.sleep(0.1)
    
    #Initialize the change in angle from gyroscope
    gyroX = 0
    gyroY = 0
    gyroZ = 0
    steady = 0
    Yaw = 0
    
    offsets = [[ 1.00089375, -0.08028328], [ 0.99821309, -0.02526564], [0.98390081, 0.05836787],
               -1.0134735107421875, -2.77301025390625, 0.46563720703125]

    for i in range(0,101):
        G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
        A_xx = fiteqn(A_x, *offsets[0])
        A_yy = fiteqn(A_y, *offsets[1])
        A_zz = fiteqn(A_z, *offsets[2])
        G_xx = G_x - offsets[3]
        G_yy = G_y - offsets[4]
        G_zz = G_z - offsets[5]
        if avgcount < 100:
            avgcalc[0] = np.append(avgcalc[0], A_xx)
            avgcalc[1] = np.append(avgcalc[1], A_yy)                    
            
        elif avgcount == 100:
           
            avgoffset[0] = np.mean(avgcalc[0])
            avgoffset[1] = np.mean(avgcalc[1])
#         
        avgcount +=1
        print("\rCount:", avgcount)
        

    while True:
        stdscr.nodelay(True)
        c = stdscr.getch() 
        if c == curses.KEY_LEFT:
            stdscr.addstr('left\n')
            curses.flushinp()
            out[0] -= 10
            out[1] -= 10
            out[2] += 10
            out[3] += 10
            out[4] -= 10    
            out[5] += 10
            out = np.clip([out[0], out[1], out[2], out[3], out[4], out[5]], 0, 180)
            steady = 0
        elif c == curses.KEY_RIGHT:
            stdscr.addstr('right\n')
            curses.flushinp()
            out[0] += 10
            out[1] += 10
            out[2] -= 10
            out[3] -= 10
            out[4] += 10    
            out[5] -= 10
            out = np.clip([out[0], out[1], out[2], out[3], out[4], out[5]], 0, 180)
            steady = 0
        elif c == curses.KEY_UP:
            stdscr.addstr('up\n')
            curses.flushinp()
            out[0] += 10
            out[1] -= 10
            out[2] -= 10
            out[3] += 10
            out[4] -= 10    
            out[5] -= 10
            out = np.clip([out[0], out[1], out[2], out[3], out[4], out[5]], 0, 180)
            steady = 0
        elif c == curses.KEY_DOWN:
            stdscr.addstr('down\n')
            curses.flushinp()
            out[0] -= 10
            out[1] += 10
            out[2] += 10
            out[3] -= 10
            out[4] += 10    
            out[5] += 10
            out = np.clip([out[0], out[1], out[2], out[3], out[4], out[5]], 0, 180)
            steady = 0
        elif c == curses.KEY_BACKSPACE:
            spechr= '"!@#$%^&*()-+?_=<>/"'
            flag = 0
            curses.echo()
            stdscr.nodelay(False)
            while flag != 1:
                stdscr.clear()
                stdscr.addstr("Enter the angle for Servo 0,1,3,8,10,11 that is desired:")
                angles = stdscr.getstr(1,0,25)
                angles = angles.decode('UTF-8')
                if "." in angles:
                    print('\n\rYour input is not accepted! Please enter 6 integers between 1 and 180 separated by commas.')
                    time.sleep(2)
                    continue
                elif any(x in spechr for x in angles):
                    print('\n\rYour input is not accepted! Please enter 6 integers between 1 and 180 separated by commas.')
                    time.sleep(2)
                    continue                    
                splitangles = [int(x) for x in angles.split(',')]
                out = splitangles
                if len(out)!=6 or any(i > 180 for i in out) or any(i < 0 for i in out):
                    print('\n\rYour input is not accepted! Please enter 6 integers between 1 and 180 separated by commas.')
                    time.sleep(2)
                else: 
                    flag = 1
        elif c == curses.KEY_END:
            TotalRoll = 0
            TotalPitch = 0
            for i in range(0,100):
                G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
                A_xx = fiteqn(A_x, *offsets[0]) - avgoffset[0]
                A_yy = fiteqn(A_y, *offsets[1]) - avgoffset[1]
                A_zz = fiteqn(A_z, *offsets[2])
                G_xx = G_x - offsets[3]
                G_yy = G_y - offsets[4]
                G_zz = G_z - offsets[5]
                Roll, Pitch, Yaw, _, _, _, _ = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timen, steady)
                TotalRoll += Roll
                TotalPitch += Pitch
            print('\n\rRoll: ', TotalRoll/100, '\n\rPitch: ', TotalPitch/100)
            time.sleep(3)
        elif c == curses.KEY_DC:
            stdscr.clear()
            stdscr.nodelay(False)
            curses.echo()
            stdscr.addstr("\rEnter the Roll angle that is desired:")
            stdscr.refresh()
            curses.flushinp()
            rinput = stdscr.getstr(1,0,3)
            setroll = float(rinput.decode('UTF-8'))
            curses.flushinp()
            stdscr.addstr("\rEnter the Pitch angle that is desired:")
            pinput = stdscr.getstr(3,0,3)
            setpitch = float(pinput.decode('UTF-8'))
            print("rinput and pinput: ", setroll, setpitch)
            curses.flushinp()
            out, out1, out2, out3, out4 = setorien(setroll, setpitch)            
        else:
            G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
            if time.time() - pasttime > 5:
                pasttime = time.time()   
                A_xx = fiteqn(A_x, *offsets[0]) - avgoffset[0]
                A_yy = fiteqn(A_y, *offsets[1]) - avgoffset[1]
                A_zz = fiteqn(A_z, *offsets[2])
                G_xx = G_x - offsets[3]
                G_yy = G_y - offsets[4]
                G_zz = G_z - offsets[5]
                Roll, Pitch, gyroX, gyroY, gyroZ = autocalib(A_xx, A_yy, A_zz, gyroX, gyroY, gyroZ)
            steady = 1
            stdscr.clear()

        
        kit.servo[0].angle = out[0]
        kit.servo[1].angle = out[1]
        kit.servo[3].angle = out[2]
        kit.servo[8].angle = out[3]
        kit.servo[10].angle = out[4]
        kit.servo[11].angle = out[5]
        
        G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
        
        stdscr.clear()
        stdscr.refresh()
        
        A_xx = fiteqn(A_x, *offsets[0]) - avgoffset[0]
        A_yy = fiteqn(A_y, *offsets[1]) - avgoffset[1]
        A_zz = fiteqn(A_z, *offsets[2])
        G_xx = G_x - offsets[3]
        G_yy = G_y - offsets[4]
        G_zz = G_z - offsets[5]
        
        Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, timen = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timen, steady)
        print('Use the arrow keys to move the platform.\n\r')
        print('Press Backspace to enter servo motor angles manually.\n\r')
        print('Press the End key to get an average of the current value and freeze the terminal for 3 seconds.\n\r')
        print('Press the Delete key to enter the desired orientation.')
        print('\n\rRoll = ', round(Roll, 2), '\n\rPitch = ', round(Pitch,2))
        print("\n\rCurrent angle:", out)
 
if __name__ == '__main__':
    curses.wrapper(main)
    
curses.endwin()    
    
    
    
    

