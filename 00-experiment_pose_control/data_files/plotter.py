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

file = np.loadtxt(sys.argv[1], skiprows=1, usecols=range(47))

robot_q = file[:, 0:7]
robot_dq = file[:, 7:14]
x_desired = file[:, 14:17]
x_current = file[:, 17:20]
vel_desired = file[:, 20:23]
vel_current = file[:, 23:26]
ori_error = file[:, 26:29]
angvel_desired = file[:, 29:32]
angvel_current = file[:, 32:35]
posori_task_force = file[:, 35:41]
posori_task_torques = file[:, 41:48]
command_torques = file[:, 48:55]

plt.figure(1)
plt.plot(ori_error)
plt.title("orientation error")
plt.ylim(-0.08, 0.08)

plt.figure(2)
plt.plot(x_current - x_desired)
# plt.plot(x_desired, '--')
plt.title("robot position error")
plt.legend(['x','y','z'])
plt.ylim(-0.1, 0.1)

# plt.figure(3)
# plt.plot(posori_task_force[:,0:3])
# plt.title("Command force trans")

# plt.figure(4)
# plt.plot(posori_task_force[:,3:6])
# plt.title("Command force rot")

plt.show()


