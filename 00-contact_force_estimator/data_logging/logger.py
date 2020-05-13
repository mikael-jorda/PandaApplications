#!/usr/bin/env python3

# read redis keys and dump them to a file
import redis, time, signal, sys
import numpy as np
import os
import json

runloop = True
counter = 0

if len(sys.argv) < 2:
    print("Give the prefix of the file to write as an argument\n")
    exit()


# handle ctrl-C and close the files
def signal_handler(signal, frame):
    global runloop
    runloop = False
    print(' ... Exiting data logger')

signal.signal(signal.SIGINT, signal_handler)

# data files
folder = 'test/'

if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file prefix
prefix = sys.argv[1]
# prefix = 'test'

# open files
file = open(folder + '/' + prefix + '_' + timestamp,'w')

file.write('q[7]\tdq[7]\tddq[7]\tq_kal[7]\tdq_kal[7]\tddq_kal[7]\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)
pipe = r_server.pipeline(False)

JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
JOINT_ACCELERATIONS_KEY = "sai2::PandaApplication::sensors::ddq";

JOINT_ANGLES_KALMAN_KEY  = "sai2::PandaApplication::kalman_filter::q";
JOINT_VELOCITIES_KALMAN_KEY = "sai2::PandaApplication::kalman_filter::dq";
JOINT_ACCELERATIONS_KALMAN_KEY = "sai2::PandaApplication::kalman_filter::ddq";

JOINT_VELOCITIES_FILTERED_KEY = "sai2::PandaApplication::butterworth_filter::dq";
JOINT_ACCELERATIONS_FILTERED_KEY = "sai2::PandaApplication::butterworth_filter::ddq";

# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
    t += logger_period


    pipe.get(JOINT_ANGLES_KEY)
    pipe.get(JOINT_VELOCITIES_KEY)
    pipe.get(JOINT_ACCELERATIONS_KEY)
    pipe.get(JOINT_ANGLES_KALMAN_KEY)
    pipe.get(JOINT_VELOCITIES_KALMAN_KEY)
    pipe.get(JOINT_ACCELERATIONS_KALMAN_KEY)
    pipe.get(JOINT_VELOCITIES_FILTERED_KEY)
    pipe.get(JOINT_ACCELERATIONS_FILTERED_KEY)

    responses = pipe.execute()

    line = "";

    for response in responses:
        # print(response)
        r_array = np.array(json.loads(response.decode("utf-8")))

        if(len(np.shape(r_array)) == 0):
            line += str(r_array) + '\t'
        elif(len(np.shape(r_array)) == 1):
            line += " ".join([str(x) for x in r_array]) + '\t'
        elif(len(np.shape(r_array)) == 2):
            r_array_row = np.resize(r_array, (1, np.shape(r_array)[0]*np.shape(r_array)[1]))[0]
            line += " ".join([str(x) for x in r_array_row]) + '\t'
    line += '\n'

    file.write(line)

    counter = counter + 1

    time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
