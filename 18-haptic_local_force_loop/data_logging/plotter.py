#!/usr/bin/env python

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys

# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

file = np.loadtxt(sys.argv[1] ,skiprows=1, delimiter=",")

time = file[:,0]/1000000
robot_position = file[:,1:4]
robot_velocity = file[:,4:7]
haptic_position = file[:,7:10]
haptic_velocity = file[:,10:13]
robot_force = file[:,13:16]
haptic_force = file[:,16:19]
sensed_force = file[:,19:22]
eigenvalues = file[:,22:25]
evec0 = file[:,25:28]
evec1 = file[:,28:31]
evec2 = file[:,31:34]

force_axis = file[:,34:37]
motion_axis = file[:,37:40]


plt.figure(1)
plt.subplot(3,1,1)
plt.plot(time,robot_position[:,0], 'r')
plt.plot(time,haptic_position[:,0], 'b--')
plt.title("robot and haptic position")
plt.subplot(3,1,2)
plt.plot(time,robot_position[:,1], 'r')
plt.plot(time,haptic_position[:,1], 'b--')
plt.subplot(3,1,3)
plt.plot(time,robot_position[:,2], 'r')
plt.plot(time,haptic_position[:,2], 'b--')


plt.figure(2)
plt.subplot(3,1,1)
plt.plot(time, -robot_force[:,0], 'r')
plt.plot(time, haptic_force[:,0], 'b--')
plt.plot(time, sensed_force[:,0], 'g')
# plt.plot(time, filtered_sensed_force[:,0], 'g')
plt.ylim([-10,10])
plt.title("robot, haptic and sensed forces")
plt.subplot(3,1,2)
plt.plot(time, -robot_force[:,1], 'r')
plt.plot(time, haptic_force[:,1], 'b--')
plt.plot(time, sensed_force[:,1], 'g')
# plt.plot(time, filtered_sensed_force[:,1], 'g')
plt.ylim([-10,10])
plt.subplot(3,1,3)
plt.plot(time, -robot_force[:,2], 'r')
plt.plot(time, haptic_force[:,2], 'b--')
plt.plot(time, sensed_force[:,2], 'g')
# plt.plot(time, filtered_sensed_force[:,2], 'g')
plt.ylim([-10,10])


plt.show()


