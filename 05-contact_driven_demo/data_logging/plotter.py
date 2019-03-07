#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data file to read given as argument
if len(sys.argv) < 2:
	print("Give the name of the file to read as an argument\n")
	exit()

file = np.loadtxt(sys.argv[1] ,skiprows=1)

time = file[1::,0]
r_mom = file[1::,1:8]
r_filtered = file[1::,8:15]
tau_comp = file[1::,15:22]
tau_cmd = file[1::,22:29]
x_des = file[1::,29:32]
x_curr = file[1::,32:35]
f_contact = file[1::,35:38]
q_des = file[1::,38:45]
q_curr = file[1::,45:52]

f_contact_bis = f_contact
for i in range(1,np.shape(r_mom)[0]-1):
	f_contact_bis[i,:] = 0.5*(f_contact[i-1,:] + f_contact[i+1,:])


time = np.arange(np.shape(r_mom)[0])
time = time/100

# print(np.sqrt(np.mean((x_des-x_curr).dot((x_des - x_curr).T))))
# print((x_des-x_curr)[-1,:])

# r_norm = np.linalg.norm(r_filtered, axis=1)
# plt.figure(5)
# plt.plot(time, r_norm)
# plt.plot(time, 0.3*np.ones(np.size(time)))
# plt.title("Norm of contact observer")

# plt.figure(1, figsize=(7,8))
# # plt.subplot(211)
# # plt.plot(time,r_mom)
# # plt.title("velocity based observer")
# # plt.subplot(212)
# plt.plot(time,r_filtered)
# plt.plot(time, 0.15*np.ones(np.size(time)), color='k')
# plt.plot(time, -0.15*np.ones(np.size(time)), color='k')
# plt.title("momentum based observer filtered")

fig = plt.figure(2)
# fig = plt.figure(2, figsize=(5,3.5))
fig.subplots_adjust(hspace=0.25)
plt.subplot(311)
# plt.title("Cartesian Position Error",fontsize='20')
plt.plot(time,x_des[:,0]-x_curr[:,0], c='r')
plt.plot(time,0.005*np.ones(np.size(time)), 'k--')
plt.plot(time,-0.005*np.ones(np.size(time)), 'k--')
plt.axis([time[1], time[-1], -0.019, 0.01])
# plt.axis([time[1], time[-1], -0.015, 0.03])
plt.gca().axes.get_xaxis().set_visible(False)
plt.ylabel("x", fontsize="18")
plt.subplot(312)
plt.plot(time,x_des[:,1]-x_curr[:,1], c='g')
plt.plot(time,0.005*np.ones(np.size(time)), 'k--')
plt.plot(time,-0.005*np.ones(np.size(time)), 'k--')
# plt.axis([time[1], time[-1], -0.01, 0.022])
# plt.axis([time[1], time[-1], -0.02, 0.022])
plt.gca().axes.get_xaxis().set_visible(False)
plt.ylabel("y", fontsize="18")
plt.subplot(313)
plt.plot(time,x_des[:,2]-x_curr[:,2], c='b')
plt.plot(time,0.005*np.ones(np.size(time)), 'k--')
plt.plot(time,-0.005*np.ones(np.size(time)), 'k--')
plt.axis([time[1], time[-1], -0.007, 0.007])
# plt.axis([time[1], time[-1], -0.018, 0.023])
plt.ylabel("z", fontsize="18")
plt.xlabel("time (s)", fontsize="18")

# plt.figure(3, figsize=(5,4))

# plt.figure(3)
# plt.plot(time, tau_cmd)
# plt.axis([time[1], time[-1], -35, 45])
# # plt.axis([time[1], time[-1], -45, 35])
# plt.xlabel("time (s)", fontsize="18")
# plt.ylabel("Control Torques (Nm)", Fontsize="18")


# plt.axis([time[1], time[-1], -2, 10])
# plt.title("Torques commanded")

f_norm = np.linalg.norm(f_contact_bis, axis=1)
for i in range(1,np.size(f_norm)-1):
	if(abs(f_norm[i-1] - f_norm[i]) > 3*abs(f_norm[i-1] - f_norm[i+1])):
		f_norm[i] = (f_norm[i-1] + f_norm[i+1])/2

# plt.figure(4, figsize=(5,3))
plt.figure(4)
plt.plot(time, 1.1*f_norm, c='k')
plt.axis([time[1], time[-1], -5, 60])
# plt.title("Norm of Contact Force", fontsize='20')
plt.xlabel("time (s)", fontsize="18")
plt.ylabel("Force (N)", fontsize="18")

# plt.figure(10)
# plt.plot(time, 180/math.pi*(q_curr - q_des))
# plt.axis([time[1], time[-1], -80, 95])
# plt.xlabel("time (s)", fontsize="18")
# plt.ylabel("joint error (degrees)", fontsize="18")


# plt.title("joint posture error")

plt.show()


