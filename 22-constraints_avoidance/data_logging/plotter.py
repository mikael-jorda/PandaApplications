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

file = np.loadtxt(sys.argv[1] ,skiprows=1, delimiter=",")

time = file[:,0]/1000000
x_desired = file[:,1:4]
x_current = file[:,4:7]
sensed_force = file[:,7:10]
mom_observer = file[:,10:18]
force_elbow = file[:,18:21]
d_c = file[:,21]
d_z = file[:,22]
d_t = file[:,23]
obstacle_avoidance_force = file[:,24:27]
contact_driven_force = file[:,27:30]


median_filter_size = 100
filter_step = math.floor(median_filter_size / 2)

filtered_sensed_force = np.array(sensed_force)
filtered_elbow_force = np.array(force_elbow)

n_samples = len(time)

for i in range(filter_step + 1, n_samples - filter_step - 1):
	filtered_sensed_force[i] = np.median(sensed_force[(i - filter_step):(i +filter_step)], axis=0)
	filtered_elbow_force[i] = np.median(force_elbow[(i - filter_step):(i +filter_step)], axis=0)

norm_elbow_force = np.sqrt(np.sum(filtered_elbow_force*filtered_elbow_force, axis=1))

plt.figure(1)
plt.plot(x_current[:,1], x_current[:,0])
plt.plot(x_desired[:,1], x_desired[:,0])
plt.title("trajectory")

plt.figure(3)
plt.plot(filtered_sensed_force)
plt.title("sensed force")

plt.figure(4)
plt.plot(norm_elbow_force)
plt.title("elbow force")

plt.figure(5)
plt.plot(d_c)
plt.plot(d_z)
plt.plot(d_t)
plt.title("distances to obstacle")

plt.show()


