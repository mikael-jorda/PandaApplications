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

desired_force = file[1::, 0]
sensed_force = file[1::, 1]
Rc = file[1::, 2]
forward_PO = file[1::, 3]
backward_PO = file[1::, 4]
E_correction_forward = file[1::, 5]
E_correction_backward = file[1::, 6]
vc = file[1::, 7]
alpha = file[1::, 8]

# window = range(3000)
window = range(len(Rc))

time = np.array(range(len(Rc[window])))/1000.0

plt.figure(1)
plt.plot(time, sensed_force[window], label='Fs')
plt.plot(time, desired_force[window], 'r--', label='Fd')
plt.title("desired vs sensed force")
plt.legend()

plt.figure(10)
plt.plot(time, vc)
plt.title("vc")

plt.figure(2)
plt.plot(time, Rc)
plt.title("Rc")

# plt.figure(12)
# plt.plot(time, alpha)
# plt.title("alpha")

plt.figure(3)
plt.plot(time,backward_PO)
plt.title("backward PO")

plt.figure(4)
plt.plot(time,E_correction_backward)
plt.title("E correction backward")

# plt.figure(5)
# plt.plot(time,forward_PO)
# plt.title("forward PO")

# plt.figure(6)
# plt.plot(time,E_correction_forward)
# plt.title("E correction forward")

plt.show()


