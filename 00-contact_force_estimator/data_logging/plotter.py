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

q = file[1::,0:7]
dq = file[1::,7:14]
ddq = file[1::,14:21]
q_kalman_cpp = file[1::,21:28]
dq_kalman_cpp = file[1::,28:35]
ddq_kalman_cpp = file[1::,35:42]
dq_filtered_cpp = file[1::,42:49]
ddq_filtered_cpp = file[1::,49:56]

q_kalman = 0.0 * q
q_kalman[0,:] = q[0,:]
dq_kalman = 0.0 * q
ddq_kalman = 0.0 * q

x_hat = np.zeros((21))

x_hat[0:7] = q_kalman[0,:]
x_hat[7:14] = dq_kalman[0,:]
x_hat[14:21] = ddq_kalman[0,:]

# kalman filter
dt = 0.005
mu_process = 0.0;
sigma_process_1 = 0;
sigma_process_2 = 0;
sigma_process_3 = 100;
mu_meas = 0.0;
sigma_meas = 0.1;
F = np.eye(21)
F[0:7,7:14] = dt * np.eye(7)
F[7:14,14:21] = dt * np.eye(7)

H = np.zeros((7,21))
H[0:7,0:7] = np.eye(7)

Q = np.zeros((21,21))
Q[0:7,0:7] = sigma_process_1 * np.eye(7)
Q[7:14,7:14] = sigma_process_2 * np.eye(7)
Q[14::,14::] = sigma_process_3 * np.eye(7)
R = sigma_meas * np.eye(7)

P = 0.0 * np.eye(21)

n_steps = len(q[:,0])

for i in range(n_steps-1):

	z = q[i+1,:]

	# print((H.dot(P.dot(H.T)) + R))

	K = P.dot(H.T).dot(np.linalg.inv(H.dot(P.dot(H.T)) + R))
	x_hat = x_hat + K.dot(z - H.dot(x_hat))
	P = np.array(np.eye(21) - K.dot(H)).dot(P)

	q_kalman[i+1,:] = x_hat[0:7]
	dq_kalman[i+1,:] = x_hat[7:14]
	ddq_kalman[i+1,:] = x_hat[14:21]

	x_hat = F.dot(x_hat)
	P = F.dot(P.dot(F.T)) + Q




# print(Q)


plt.figure(0)
plt.plot(q,'k')
# plt.plot(q_kalman, 'r-.')
plt.plot(q_kalman_cpp, 'r-.')
plt.title("q")

plt.figure(1)
plt.plot(dq,'k')
# plt.plot(dq_kalman, 'r-.')
plt.plot(dq_kalman_cpp, 'r-.')
plt.title("dq")

plt.figure(2)
plt.plot(ddq,'k')
# plt.plot(ddq_kalman, 'r-.')
plt.plot(ddq_kalman_cpp, 'r-.')
plt.title("ddq")

plt.figure(3)
plt.plot(dq,'k')
# plt.plot(dq_kalman, 'r-.')
plt.plot(dq_filtered_cpp, 'r-.')
plt.title("dq filtered")

plt.figure(4)
plt.plot(ddq,'k')
# plt.plot(ddq_kalman, 'r-.')
plt.plot(ddq_filtered_cpp, 'r-.')
plt.title("ddq filtered")

# plt.figure(10)
# plt.plot(q_kalman - q_kalman_cpp)

# plt.figure(0)
# plt.plot(q - q_kalman)
# # plt.plot(q_kalman, '--')
# plt.title("q - q_kalman")

# # plt.figure(10)
# # plt.plot(q_kalman)
# # plt.title("q kalman")

# plt.figure(1)
# plt.plot(dq_driver - dq_kalman)
# # plt.plot(dq_kalman, '--')
# plt.title("dq - dq_kalman")

# # plt.figure(11)
# # plt.plot(dq_kalman)
# # plt.title("dq kalman")

# plt.figure(2)
# plt.plot(ddq)
# # plt.plot(ddq_kalman, '--')
# plt.title("ddq")

# plt.figure(12)
# plt.plot(ddq_kalman)
# plt.title("ddq kalman")

# plt.figure(5)
# plt.plot(dq_diff, '--')
# plt.title("dq diff")

# plt.figure(6)
# plt.plot(ddq_diff, '--')
# plt.title("ddq diff")

plt.show()


