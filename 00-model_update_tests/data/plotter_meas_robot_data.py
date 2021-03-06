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

# file = np.loadtxt(sys.argv[1], skiprows=1, usecols=range(47))
file = np.loadtxt(sys.argv[1], skiprows=1)

robot_q = file[:, 0:7]
robot_dq = file[:, 7:14]
x_current = file[:, 14:17]
vel_current = file[:, 17:20]
angvel_current = file[:, 20:23]

mass_matrix = file[:, 23:72]
task_jacobian = file[:, 72:114]


m_init = np.reshape(mass_matrix[0,:],(7,7))
J_init = np.reshape(task_jacobian[0,:],(7,6)).T

print(m_init)
print()
print(J_init)
print()
print(np.linalg.inv(J_init.dot(np.linalg.inv(m_init).dot(J_init.T))))



# plt.figure(1)
# plt.plot(ori_error)
# plt.title("orientation error")
# # plt.ylim(-0.08, 0.08)

# plt.figure(2)
# plt.plot(x_current - x_desired)
# # plt.plot(x_desired, '--')
# plt.title("robot position error")
# plt.legend(['x','y','z'])
# # plt.ylim(-0.1, 0.1)

# # plt.figure(3)
# # plt.plot(posori_task_force[:,0:3])
# # plt.title("Command force trans")

# # plt.figure(4)
# # plt.plot(posori_task_force[:,3:6])
# # plt.title("Command force rot")

# plt.show()


