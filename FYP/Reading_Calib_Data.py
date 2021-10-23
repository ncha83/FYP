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
def resetplat():
    kit.servo[0].angle = 0
    kit.servo[1].angle = 180
    kit.servo[3].angle = 0
    kit.servo[8].angle = 180
    kit.servo[10].angle = 0
    kit.servo[11].angle = 180
    
    return

def setplat(a0, a1, a3, a8, a10, a11):
    kit.servo[0].angle = a0
    kit.servo[1].angle = a1
    kit.servo[3].angle = a3
    kit.servo[8].angle = a8
    kit.servo[10].angle = a10
    kit.servo[11].angle = a11
    
    return

def fiteqn(m_x, x_input, b):
    return (m_x*x_input)+b

def autocalib(gconfig, aconfig):
    #How would this work if it was not at home position?
    count = 0
    offsets = [[],[],[]]
    data = [[],[],[]]
    
    while count < 50:
        _, _, _, A_x, A_y, A_z= sensout(gconfig, aconfig)
        data[0] = np.append(data[0], A_x)
        data[1] = np.append(data[1], A_y)
        data[2] = np.append(data[2], A_z)
        count += 1
    
    Xpopt, _ = curve_fit(fiteqn, data[0], 0*np.ones(np.shape(data[0])))
    Ypopt, _ = curve_fit(fiteqn, data[1], 0*np.ones(np.shape(data[1])))
    Zpopt, _ = curve_fit(fiteqn, data[2], 1*np.ones(np.shape(data[2])))
    
    offsets[0] = Xpopt
    offsets[1] = Ypopt
    offsets[2] = Zpopt
    
    return offsets

def oriencalc(Ax, Ay, Az):
    #Roll is front/back axis
    #Pitch is left/right axis
    #Yaw is up/down axis
    float roll = m.degrees(m.atan(Ay/sqrt(pow(Ax,2) + pow(Az,2))))
    float pitch = m.degrees(m.atan(Ax/sqrt(pow(Ax,2) + pow(Az,2))))
    
    return roll, pitch



#Main code
def main(stdscr):
    time.sleep(0.1)
    resetplat()

    #Get gyro and accel config vals
    gconfig, aconfig = IMU_start()
    time.sleep(0.1)
    
    offsets = autocalib(gconfig, aconfig)

    while True:
        stdscr.nodelay(True)
        c = stdscr.getch()
        
        angle0 = 0
        angle1 = 180
        angle3 = 0
        angle8 = 180
        angle10 = 0
        angle11 = 180
        
        _, _, _, A_x, A_y, A_z = sensout(gconfig, aconfig)
        
        resetplat()
        
        for i in range(0,19):
            angle0 = i*10
            setplat(angle0, angle1, angle3, angle8, angle10, angle11)
            for ii in range(0,19):
                angle1 = 180 - ii*10
                setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                
                for iii in range(0,19):
                    angle3 = iii*10
                    setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                    
                    for iiii in range(0,19):
                        angle8 = 180 - iiii*10
                        setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                        
                        for iiiii in range(0,19):                            
                            angle10 = iiiii*10
                            setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                            
                            for iiiiii in range(0,19):                                
                                angle11 = 180 - iiiiii*10
                                setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                                
        #Array is [angle0, angle1, angle3, angle8, angle10, angle11, ]
        dataAsInt = [angle0, angle1, angle3, angle8, angle10, angle11, A_xx, A_yy, A_zz, roll, pitch]
        dataAsString = str(dataAsInt)

        fb = open('/home/pi/Desktop/cailbration_data','a+')
        fb.write(dataAsString) 
        fb.write('\n')
        fb.close()
                                
        
        #0 to 180
        kit.servo[0].angle = angle0
        #180 to 0
        kit.servo[1].angle = angle1
        #0 to 180
        kit.servo[3].angle = angle3
        #180 to 0
        kit.servo[8].angle = angle8
        #0 to 180
        kit.servo[10].angle = angle10
        #180 to 0
        kit.servo[11].angle = angle11
        
        G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
        #time.sleep(0.1)
        
        stdscr.clear()
        stdscr.refresh()
        print('\n\rG_x', G_x, '\n\rG_y', G_y, '\n\rG_z', G_z, '\n\rA_x', A_x, '\n\rA_y', A_y, '\n\rA_z', A_z)
        
        TotA = m.sqrt(A_x**2 + A_y**2 + A_z**2)
        print('\n\rTotA = ', TotA)
        
        A_xx = fiteqn(A_x, *offsets[0])
        A_yy = fiteqn(A_y, *offsets[1])
        A_zz = fiteqn(A_z, *offsets[2])
        
        print('\n\rA_xx', A_xx, '\n\rA_yy', A_yy, '\n\rA_zz', A_zz)
        
        TotA = m.sqrt(A_xx**2 + A_yy**2 + A_zz**2)
        print('\n\rTotA = ', TotA)

 
if __name__ == '__main__':
    curses.wrapper(main)
    
curses.endwin()    
    
    
    
    


