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

def fiteqn(m_x,x_input, b):
    return (m_x*x_input)+b

def accelcalib(gconfig, aconfig):
    #How would this work if it was not at home position?
    count = 0
    offsets = [[],[],[]]
    data = [[],[],[]]
    
    
    for ii in range (0,3):
        print("\n\rPress Enter to continue")
        c = stdscr.getch()
        count = 0
        while count < 1000:
            _, _, _, A_x, A_y, A_z= sensout(gconfig, aconfig)
            data[0] = np.append(data[0], A_x)
            count += 1
            
    for ii in range (0,3):
        print("\n\rPress Enter to continue")
        c = stdscr.getch()
        count = 0
        while count < 1000:
            _, _, _, A_x, A_y, A_z= sensout(gconfig, aconfig)
            data[1] = np.append(data[1], A_y)
            count += 1
            
    for ii in range (0,3):
        print("\n\rPress Enter to continue")        
        c = stdscr.getch()
        count = 0
        while count < 1000:
            _, _, _, A_x, A_y, A_z= sensout(gconfig, aconfig)
            data[2] = np.append(data[2], A_z)
            count += 1
                                
    Xpopt, _ = curve_fit(fiteqn, data[0], np.append(np.append(1*np.ones(1000),-1*np.ones(1000)),0*np.ones(1000)))
    Ypopt, _ = curve_fit(fiteqn, data[1], np.append(np.append(1*np.ones(1000),-1*np.ones(1000)),0*np.ones(1000)))
    Zpopt, _ = curve_fit(fiteqn, data[2], np.append(np.append(1*np.ones(1000),-1*np.ones(1000)),0*np.ones(1000)))
    
    offsets[0] = Xpopt
    offsets[1] = Ypopt
    offsets[2] = Zpopt
    
    return offsets

def oriencalc(Ax, Ay, Az):
    #Roll is front/back axis
    #Pitch is left/right axis
    #Yaw is up/down axis
    roll = m.degrees(m.atan(Ay/m.sqrt(pow(Ax,2) + pow(Az,2))))
    pitch = m.degrees(m.atan(Ax/m.sqrt(pow(Ay,2) + pow(Az,2))))
    
    return roll, pitch



#Main code
def main(stdscr):
    time.sleep(0.1)
    angle0 = 80
    angle1 = 90
    angle3 = 90
    angle8 = 95
    angle10 = 115
    angle11 = 95
    resetplat(angle0, angle1, angle3, angle8, angle10, angle11)

    #Get gyro and accel config vals
    gconfig, aconfig = IMU_start()
    time.sleep(0.1)
    
#     offsets = accelcalib(gconfig, aconfig)
#     
#     dataAsInt = offsets
#     dataAsString = str(dataAsInt)
#     
#     fb = open('/home/pi/Desktop/offsets_used_2','a+')
#     fb.write(dataAsString) 
#     fb.write('\n')
#     fb.close()
#          
        
    data = np.array([sensout(gconfig, aconfig) for ii in range(0,100)])
    
    offsets = [[ 1.00089375, -0.08028328], [ 0.99821309, -0.02526564], [0.98390081, 0.05836787]]
    offsets2 = [[ 1.0070142 , -0.07586297], [ 1.00219487, -0.01387356], [0.9808182 , 0.07008684]]
    
    accel_labels = ['a_x', 'a_y', 'a_z']
    plt.style.use('ggplot')
    fig, axs = plt.subplots(2,1,figsize=(12,9))
    for i in range (0,3):
        axs[0].plot(data[:,i+3], label ='${}$, Uncalibrated'.format(accel_labels[i]))
        axs[1].plot(fiteqn(data[:,i+3], *offsets[i]), label ='${}$, Calibrated'.format(accel_labels[i]))
    axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$a_{x,y,z}$ [dps]',fontsize=18)
    axs[1].set_ylabel('$a_{x,y,z}$ [dps]',fontsize=18)
    axs[1].set_xlabel('Sample',fontsize=18)
    axs[0].set_ylim([-4,2]);axs[1].set_ylim([-2,2])
    axs[0].set_title('Accelerometer Calibration Graphs',fontsize=18)    
    fig.savefig('accel_calibration_output.png',dpi=300, bbox_inches='tight',facecolor='#FCFCFC')        
    fig.show()
#     
#     fig2, axs2 = plt.subplots(2,1,figsize=(12,9))
#     for ii in range (0,3):
#         axs2[0].plot(data[:,ii+3], label ='${}$, Uncalibrated'.format(accel_labels[ii]))
#         axs2[1].plot(fiteqn(data[:,ii+3], *offsets2[ii]), label ='${}$, Calibrated'.format(accel_labels[ii]))
#     axs2[0].legend(fontsize=14);axs[1].legend(fontsize=14)
#     axs2[0].set_ylabel('$a_{x,y,z}$ [dps]',fontsize=18)
#     axs2[1].set_ylabel('$a_{x,y,z}$ [dps]',fontsize=18)
#     axs2[1].set_xlabel('Sample',fontsize=18)
#     axs2[0].set_ylim([-4,2]);axs2[1].set_ylim([-2,2])
#     axs2[0].set_title('Accelerometer Calibration Graphs',fontsize=18)    
#     fig2.savefig('accel_calibration_output_2.png',dpi=300, bbox_inches='tight',facecolor='#FCFCFC')        
#     fig2.show()

#     accel_labels = ['a_x', 'a_y', 'a_z']
#     plt.style.use('ggplot')
#     fig, axs = plt.subplots(2,1,figsize=(12,9))
#     for iii in range (0,3):
#         axs[0].plot(fiteqn(data[:,iii+3], *offsets[iii]), label ='${}$, 50 Data Points'.format(accel_labels[iii]))
#         axs[1].plot(fiteqn(data[:,iii+3], *offsets2[iii]), label ='${}$, 1000 Data Points'.format(accel_labels[iii]))
#     axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
#     axs[0].set_ylabel('$a_{x,y,z}$ [dps]',fontsize=18)
#     axs[1].set_ylabel('$a_{x,y,z}$ [dps]',fontsize=18)
#     axs[1].set_xlabel('Sample',fontsize=18)
#     axs[0].set_ylim([-2,2]);axs[1].set_ylim([-2,2])
#     axs[0].set_title('Comparative Accelerometer Calibration Graphs',fontsize=18)    
#     fig.savefig('accel_calibration_output_compared.png',dpi=300, bbox_inches='tight',facecolor='#FCFCFC')        
#     fig.show()

 
if __name__ == '__main__':
    curses.wrapper(main)
    
curses.endwin()    
    
    
    
    




