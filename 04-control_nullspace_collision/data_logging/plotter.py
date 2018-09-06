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
r_processed = file[1::,8:15]
tau_comp = file[1::,15:22]
tau_cmd = file[1::,22:29]
x_curr = file[1::,29:32]
x_des = file[1::,32:35]
f_contact = file[1::,35:38]

time = np.arange(np.shape(r_vel)[0])
time = time/100

# print(np.sqrt(np.mean((x_des-x_curr).dot((x_des - x_curr).T))))
# print((x_des-x_curr)[-1,:])

plt.figure(1, figsize=(7,8))
plt.subplot(211)
plt.plot(time,r_vel)
plt.title("velocity based observer")
plt.subplot(212)
plt.plot(time,r_processed)
plt.title("processed_observer")

plt.figure(2, figsize=(5,3))
plt.subplot(311)
plt.plot(time,x_des[:,0]-x_curr[:,0], c='r')
plt.axis([time[1], time[-1], -0.01, 0.01])
plt.title("x error")
plt.subplot(312)
plt.plot(time,x_des[:,1]-x_curr[:,1], c='g')
plt.axis([time[1], time[-1], -0.02, 0.02])
plt.subplot(313)
plt.plot(time,x_des[:,2]-x_curr[:,2], c='b')
plt.axis([time[1], time[-1], -0.01, 0.01])

plt.figure(3, figsize=(5,4))
plt.plot(time, tau_cmd)
plt.axis([time[1], time[-1], -2, 10])
plt.title("Torques commanded")

# f_norm = np.linalg.norm(f_contact, axis=1)
# plt.figure(4)
# plt.plot(time, f_norm)
# plt.title("Norm of contact force")

plt.show()


