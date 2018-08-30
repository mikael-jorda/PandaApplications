#!/usr/bin/env python3

# read redis keys and dump them to a file
import redis, time, signal, sys
import os
import json

runloop = True
counter = 0

# handle ctrl-C and close the files
def signal_handler(signal, frame):
	global runloop
	runloop = False
	print(' ... Exiting data logger')

signal.signal(signal.SIGINT, signal_handler)

# data files
folder = 'debug/'
if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file names
name = "data_file"

# open files
file = open(folder + '/' + name + '_' + timestamp,'w')

file.write('timestamp\tvelocity based observer\tmomentum based observer\tcontact compensation torques\t' +\
	'command torques\tdesired position\tcurrent position\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys used in SAI2
SIM_TIMESTAMP_KEY = "sai2::PandaApplication::simulation::timestamp";
VEL_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::velocity_based_observer";
MOM_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::momentum_based_observer";
CONTACT_COMPENSATION_TORQUES_KEY = "sai2::PandaApplication::controller::logging::contact_compensation_torques";
COMMAND_TORQUES_LOGGING_KEY = "sai2::PandaApplication::controller::logging::command_torques";
DESIRED_POS_KEY = "sai2::PandaApplication::controller::logging::desired_position";
CURRENT_POS_KEY = "sai2::PandaApplication::controller::logging::current_position";

r_server.set(VEL_OBSERVER_LOGGING_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(MOM_OBSERVER_LOGGING_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(CONTACT_COMPENSATION_TORQUES_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(COMMAND_TORQUES_LOGGING_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(DESIRED_POS_KEY, json.dumps([0,0,0]))
r_server.set(CURRENT_POS_KEY, json.dumps([0,0,0]))

# data logging frequency
logger_frequency = 100.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period

	timestamp = json.loads(r_server.get(SIM_TIMESTAMP_KEY))
	r_vel = json.loads(r_server.get(VEL_OBSERVER_LOGGING_KEY))
	r_mom = json.loads(r_server.get(MOM_OBSERVER_LOGGING_KEY))
	tau_comp = json.loads(r_server.get(CONTACT_COMPENSATION_TORQUES_KEY))
	tau_cmd = json.loads(r_server.get(COMMAND_TORQUES_LOGGING_KEY))
	x_des = json.loads(r_server.get(DESIRED_POS_KEY))
	x_curr = json.loads(r_server.get(CURRENT_POS_KEY))

	line = \
	str(timestamp) + '\t' + \
	" ".join([str(x) for x in r_vel]) + '\t' + \
	" ".join([str(x) for x in r_mom]) + '\t' + \
	" ".join([str(x) for x in tau_comp]) + '\t' +\
	" ".join([str(x) for x in tau_cmd]) + '\t' +\
	" ".join([str(x) for x in x_des]) + '\t' +\
	" ".join([str(x) for x in x_curr]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
