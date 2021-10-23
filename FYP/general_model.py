import numpy as np
import math as m
from scipy.optimize import curve_fit

txt = open('/home/pi/Desktop/calibration_data_45_rest','r')
lines = txt.readlines()
length = len(lines)
print("Length: ", length)

angle0 = []
angle1 = []
angle3 = []
angle8 = []
angle10 = []
angle11 = []
roll = []
pitch = []
yaw = []

for i in range (0,length):
    data = lines[i].split(',')
    angle0 = np.append(angle0, data[1])
    angle1 = np.append(angle1, data[2])
    angle3 = np.append(angle3, data[3])
    angle8 = np.append(angle8, data[4])
    angle10 = np.append(angle10, data[5])
    angle11 = np.append(angle11, data[6])
    roll = np.append(roll, data[7])
    pitch = np.append(pitch, data[8])
    yaw = np.append(yaw, data[9])
    
def fiteqn(x_input, a, b, c, d, e, f, g):
    a0,a1,a3,a8,a10,a11 = x_input
    return a*a0 + b*a1 + c*a3 + d*a8 + e*a10 + f*a11 + g

Rpopt, _ = curve_fit(fiteqn, (angle0,angle1,angle3,angle8,angle10,angle11), roll)
Ppopt, _ = curve_fit(fiteqn, (angle0,angle1,angle3,angle8,angle10,angle11), pitch)
Ypopt, _ = curve_fit(fiteqn, (angle0,angle1,angle3,angle8,angle10,angle11), yaw)

print("Roll Offsets: ", Rpopt)
print("Pitch Offsets: ", Ppopt)
print("Yaw Offsets: ", Ypopt)

servooffsets = (Rpopt, Ppopt, Ypopt)

dataAsInt = servooffsets
dataAsString = str(dataAsInt)

fb = open('/home/pi/Desktop/offsets_used_2','a+')
fb.write(dataAsString) 
fb.write('\n')
fb.close()
