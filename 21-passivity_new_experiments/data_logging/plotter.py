#!/usr/bin/env python3

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
robot_desired_force = file[:,7:10]
robot_control_force = file[:,10:13]
sensed_force = file[:,13:16]
vc = file[:,16:19]
command_force = file[:,19:22]
PO = file[:,22]
Rc = file[:,23]
E_corr = file[:,24]



# plt.figure(1)
# plt.plot(time, robot_position)
# plt.title("robot position", FontSize=20)

# plt.figure(2)
# plt.plot(time, robot_velocity)
# plt.title("robot velocity", FontSize=20)

plt.figure(3)
plt.plot(time, robot_desired_force[:,2], 'r')
plt.plot(time, robot_control_force[:,2], 'b')
plt.plot(time, sensed_force[:,2], 'g')
# plt.plot(time, command_force[:,2], '--k')
plt.title("robot forces", FontSize=20)

plt.figure(4)
plt.plot(time, vc)
plt.title("vc", FontSize=20)

plt.figure(5)
plt.plot(time, -PO, 'r')
plt.plot(time, E_corr, 'b')
plt.title("PO", FontSize=20)

plt.figure(6)
plt.plot(time, Rc, 'r')
plt.title("Rc", FontSize=20)


plt.show()


