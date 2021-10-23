# angle0 = 0
# angle1 = 180
# angle3 = 0
# angle8 = 180
# angle10 = 0
# angle11 = 180
# 
# dataAsInt = [angle0, angle1, angle3, angle8, angle10, angle11]
# dataAsString = str(dataAsInt)
# 
# fb = open('/home/pi/Desktop/cailbration_data','a+')
# fb.write(dataAsString) 
# fb.write('\n')
# fb.close()



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

def setplat(a0, a1, a3, a8, a10, a11):
    kit.servo[0].angle = a0
    kit.servo[1].angle = a1
    kit.servo[3].angle = a3
    kit.servo[8].angle = a8
    kit.servo[10].angle = a10
    kit.servo[11].angle = a11
    
    return

def fiteqn(x_input, m_x, b):
    return (m_x*x_input)+b

def autocalib(A_x, A_y, A_z, GyroX, GyroY, GyroZ):
    
    roll = m.degrees(m.atan(Ay/m.sqrt(pow(Ax,2) + pow(Az,2))))
    pitch = m.degrees(m.atan(Ax/m.sqrt(pow(Ay,2) + pow(Az,2))))
    GyroX = 0
    GyroY = 0
    GyroZ = 0
    
    return roll, pitch, GyroX, GyroY, GyroZ

def oriencalc(Gx, Gy, Gz, gyroX, gyroY, gyroZ, Ax, Ay, Az, timenow):
    #Roll is front/back axis
    #Pitch is left/right axis
    #Yaw is up/down axis
    pasttime = timenow        
    timenow = time.time()
    timepassed = timenow - pasttime
    
    rollAcc = m.degrees(m.atan(Ay/m.sqrt(pow(Ax,2) + pow(Az,2))))
    pitchAcc = m.degrees(m.atan(Ax/m.sqrt(pow(Ay,2) + pow(Az,2))))
    
    #Integrating gyro value to get an angle
    gyroX = gyroX + Gx*timepassed
    gyroY = gyroY + Gy*timepassed
    gyroZ = gyroZ + Gz*timepassed
    
    #passing the gyro through high pass filter and accel through a low pass filter
    roll = 0*gyroX + rollAcc
    pitch = 0*gyroY + pitchAcc
    yaw = gyroZ
    
    return roll, pitch, yaw, gyroX, gyroY, gyroZ, timenow



#Main code
def main(stdscr):
    time.sleep(0.1)
    timenow = time.time()
    #out = [80, 90, 90, 90, 115, 95]
    angle0 = 80
    angle1 = 90
    angle3 = 90
    angle8 = 90
    angle10 = 115
    angle11 = 95
    resetplat(angle0, angle1, angle3, angle8, angle10, angle11)

    #Get gyro and accel config vals
    gconfig, aconfig = IMU_start()
    time.sleep(0.1)
    
    text_file = open("offsets_used", "r")
    lines = text_file.readlines()
    
    offsets = [[ 1.00089375, -0.08028328], [ 0.99821309, -0.02526564], [0.98390081, 0.05836787],
               -1.0134735107421875, -2.77301025390625, 0.46563720703125]
    
    IMUoffset = [-2.523645497951162, -1.0301913772634002]
    
    print("\n\rPress Enter to start collecting data")
    c = stdscr.getch()    

    while True:              
        for i in range(3,5):
            angle11 = i*45
            setplat(angle0, angle1, angle3, angle8, angle10, angle11)
            time.sleep(2)
            G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
            
            
            A_xx = fiteqn(A_x, *offsets[0])
            A_yy = fiteqn(A_y, *offsets[1])
            A_zz = fiteqn(A_z, *offsets[2])
            G_xx = G_x - offsets[3]
            G_yy = G_y - offsets[4]
            G_zz = G_z - offsets[5]
            
            gyroX = 0
            gyroY = 0
            gyroZ = 0
            
            Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, timenow = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timenow)
            
            dataAsInt = [angle0, angle1, angle3, angle8, angle10, angle11, Roll-IMUoffset[0], Pitch-IMUoffset[1], Yaw]
            dataAsString = str(dataAsInt)
            
            fb = open('/home/pi/Desktop/cailbration_data','a+')
            fb.write(dataAsString) 
            fb.write('\n')
            fb.close()
            
            for ii in range(0,5):
                angle10 = 180 - ii*45
                setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                time.sleep(2) 
                G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
                
                A_xx = fiteqn(A_x, *offsets[0])
                A_yy = fiteqn(A_y, *offsets[1])
                A_zz = fiteqn(A_z, *offsets[2])
                G_xx = G_x - offsets[3]
                G_yy = G_y - offsets[4]
                G_zz = G_z - offsets[5]            
                
                Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, timenow = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timenow)         
                
                dataAsInt = [angle0, angle1, angle3, angle8, angle10, angle11, Roll-IMUoffset[0], Pitch-IMUoffset[1], Yaw]
                dataAsString = str(dataAsInt)
                
                fb = open('/home/pi/Desktop/cailbration_data','a+')
                fb.write(dataAsString) 
                fb.write('\n')
                fb.close()
                
                               
                
                for iii in range(0,5):
                    angle8 = iii*45
                    setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                    time.sleep(2)
                    G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
                    
                    
                    
                    A_xx = fiteqn(A_x, *offsets[0])
                    A_yy = fiteqn(A_y, *offsets[1])
                    A_zz = fiteqn(A_z, *offsets[2])
                    G_xx = G_x - offsets[3]
                    G_yy = G_y - offsets[4]
                    G_zz = G_z - offsets[5]                   
                    
                    Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, timenow = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timenow)          
                    

                    dataAsInt = [angle0, angle1, angle3, angle8, angle10, angle11, Roll-IMUoffset[0], Pitch-IMUoffset[1], Yaw]
                    dataAsString = str(dataAsInt)
                    
                    fb = open('/home/pi/Desktop/cailbration_data','a+')
                    fb.write(dataAsString) 
                    fb.write('\n')
                    fb.close()

                    for iiii in range(0,5):
                        angle3 = 180 - iiii*45
                        setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                        time.sleep(2) 
                        G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)
                        
                        A_xx = fiteqn(A_x, *offsets[0])
                        A_yy = fiteqn(A_y, *offsets[1])
                        A_zz = fiteqn(A_z, *offsets[2])
                        G_xx = G_x - offsets[3]
                        G_yy = G_y - offsets[4]
                        G_zz = G_z - offsets[5]                   
                        
                        
                        Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, timenow = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timenow)           
                        
                        dataAsInt = [angle0, angle1, angle3, angle8, angle10, angle11, Roll-IMUoffset[0], Pitch-IMUoffset[1], Yaw]
                        dataAsString = str(dataAsInt)
                        
                        fb = open('/home/pi/Desktop/cailbration_data','a+')
                        fb.write(dataAsString) 
                        fb.write('\n')
                        fb.close()
                        
                        for iiiii in range(0,5):
                            angle1 = iiiii*45
                            setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                            time.sleep(2)  
                            G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)                                             
                            
                            A_xx = fiteqn(A_x, *offsets[0])
                            A_yy = fiteqn(A_y, *offsets[1])
                            A_zz = fiteqn(A_z, *offsets[2])
                            G_xx = G_x - offsets[3]
                            G_yy = G_y - offsets[4]
                            G_zz = G_z - offsets[5]                   
                            
                            
                            Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, timenow = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timenow)           
                            
                            
                            dataAsInt = [angle0, angle1, angle3, angle8, angle10, angle11, Roll-IMUoffset[0], Pitch-IMUoffset[1], Yaw]
                            dataAsString = str(dataAsInt)
                            
                            fb = open('/home/pi/Desktop/cailbration_data','a+')
                            fb.write(dataAsString) 
                            fb.write('\n')
                            fb.close()
                            
                            
                            
                            for iiiiii in range(0,5):                                
                                angle0 = 180 - iiiiii*45
                                setplat(angle0, angle1, angle3, angle8, angle10, angle11)
                                time.sleep(2) 
                                G_x, G_y, G_z, A_x, A_y, A_z = sensout(gconfig, aconfig)                             
                                                               
                                A_xx = fiteqn(A_x, *offsets[0])
                                A_yy = fiteqn(A_y, *offsets[1])
                                A_zz = fiteqn(A_z, *offsets[2])
                                G_xx = G_x - offsets[3]
                                G_yy = G_y - offsets[4]
                                G_zz = G_z - offsets[5]                   
                                
                                
                                Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, timenow = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timenow)           
                                
                                                            
                                
                                dataAsInt = [angle0, angle1, angle3, angle8, angle10, angle11, Roll-IMUoffset[0], Pitch-IMUoffset[1], Yaw]
                                dataAsString = str(dataAsInt)
                                
                                fb = open('/home/pi/Desktop/cailbration_data','a+')
                                fb.write(dataAsString) 
                                fb.write('\n')
                                fb.close()
                                

                                    
        #time.sleep(0.1)
        
        stdscr.clear()
        stdscr.refresh()
        print('\n\rG_x', G_x, '\n\rG_y', G_y, '\n\rG_z', G_z, '\n\rA_x', A_x, '\n\rA_y', A_y, '\n\rA_z', A_z)
        
        A_xx = fiteqn(A_x, *offsets[0])
        A_yy = fiteqn(A_y, *offsets[1])
        A_zz = fiteqn(A_z, *offsets[2])
        G_xx = G_x - offsets[3]
        G_yy = G_y - offsets[4]
        G_zz = G_z - offsets[5]           
        Roll, Pitch, Yaw, _, _, _, timenow = oriencalc(G_xx, G_yy, G_zz, gyroX, gyroY, gyroZ, A_xx, A_yy, A_zz, timenow)  
        
        print('\n\rO_xx', offsets[0], '\n\rO_yy', offsets[1], '\n\rO_zz', offsets[2])

        print('\n\rA_xx', A_xx, '\n\rA_yy', A_yy, '\n\rA_zz', A_zz)
        
        print('\n\rRoll = ', Roll, '\n\rPitch = ', Pitch)


 
if __name__ == '__main__':
    curses.wrapper(main)
    
curses.endwin()    
    
    
    
    



