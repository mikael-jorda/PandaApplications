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
# timestamp = ""

# file prefix
prefix = sys.argv[1]

# open files
# file = open(folder + '/' + prefix + '_' + timestamp + '.txt','w')
file = open(folder + '/' + prefix + '.txt','w')

file.write('desired_force[1]\tsensed_force[1]\tRc[1]\tforward_PO[1]\tbackward_PO[1]\t' +\
  'stored_energy_forward[1]\tstored_energy_backward[1]\tvc[1]\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)
pipe = r_server.pipeline(False)
# pipe = r_server.pipeline()

# redis keys used in SAI2
CONTROLLER_RUNNING_KEY = "sai2::PandaApplication::controller::flag_running";

DESIRED_EE_FORCE_KEY = "sai2::PandaApplication::controller:desried_ee_force";
SENSED_EE_FORCE_KEY = "sai2::PandaApplication::controller:sensed_ee_force";
RC_KEY = "sai2::PandaApplication::controller:Rc";
FORWARD_PO_KEY = "sai2::PandaApplication::controller:forward_PO";
BACKWARD_PO_KEY = "sai2::PandaApplication::controller:backward_PO";
CORRECTION_ENERGY_FORWARD_KEY = "sai2::PandaApplication::controller:E_correction_forward";
CORRECTION_ENERGY_BACKWARD_KEY = "sai2::PandaApplication::controller:E_correction_backward";
VC_KEY = "sai2::PandaApplication::controller:vc";
ALPHA_KEY = "sai2::PandaApplication::controller:alpha";

# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

while(runloop and json.loads(r_server.get(CONTROLLER_RUNNING_KEY)) != 1):
    continue

    # sim_time = json.loads(r_server.get(SIM_TIMESTAMP_KEY).decode("utf-8"))


print('Start Logging Data ... \n')

while(runloop):
    t += logger_period

    pipe.get(DESIRED_EE_FORCE_KEY)
    pipe.get(SENSED_EE_FORCE_KEY)
    pipe.get(RC_KEY)
    pipe.get(FORWARD_PO_KEY)
    pipe.get(BACKWARD_PO_KEY)
    pipe.get(CORRECTION_ENERGY_FORWARD_KEY)
    pipe.get(CORRECTION_ENERGY_BACKWARD_KEY)
    pipe.get(VC_KEY)
    pipe.get(ALPHA_KEY)

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

    if(json.loads(r_server.get(CONTROLLER_RUNNING_KEY)) != 1):
        runloop = False

    time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
