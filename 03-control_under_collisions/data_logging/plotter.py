#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys

# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

file = np.loadtxt(sys.argv[1] ,skiprows=1)

time = file[1::,0]
r_vel = file[1::,1:8]
r_mom = file[1::,8:15]
tau_comp = file[1::,15:22]
tau_cmd = file[1::,22:29]
x_curr = file[1::,29:32]
x_des = file[1::,32:35]

plt.figure(1, figsize=(7,8))
plt.subplot(211)
plt.plot(time,r_vel)
plt.title("velocity based observer")
plt.subplot(212)
plt.plot(time,tau_comp)
plt.title("contact comensation torques")

plt.figure(2, figsize=(5,4))
plt.plot(time,x_curr)
plt.plot(time,x_des)
plt.title("x vs xd")

plt.figure(3, figsize=(5,4))
plt.plot(time, tau_cmd)
plt.title("Torques commanded")

plt.show()


