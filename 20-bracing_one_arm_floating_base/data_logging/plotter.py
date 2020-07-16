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
command_torques = file[:,1:9]
sensed_force = file[:,9:12]
x_current = file[:,12:15]
x_desired = file[:,15:18]

x_error = x_current - x_desired;

norm1_torques = np.sum(np.abs(command_torques[:,1::]), 1)
norm2_torques = np.sqrt(np.sum(command_torques[:,1::]*command_torques[:,1::], 1))

norm2_xerror = np.sqrt(np.sum(x_error*x_error, 1))

plt.figure(1)
plt.plot(command_torques)
plt.title("Command Torques")

plt.figure(2)
plt.plot(norm1_torques)
plt.title("Total motor power")

plt.figure(4)
plt.plot(x_error[:,0])
plt.title("y error")

plt.figure(14)
plt.plot(x_current[:,0])
plt.plot(x_desired[:,0], 'r--')
plt.title("y vs y des")

plt.figure(5)
plt.plot(sensed_force)
plt.title("f sensed")

plt.show()


