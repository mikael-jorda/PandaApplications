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
# folder = 'debug/'
# folder = 'simulation/'
folder = 'simulation_redone/'
# folder = 'simulations_paper'
# folder = 'test/'
if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
timestamp = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

# file names
name = "cd_f10"
# name = "comp_kin_kp100_fc10"
# name = 'k50_comp_kin_fc5'

# open files
file = open(folder + '/' + name + '_' + timestamp,'w')

file.write('timestamp\tmomentum based observer\tfiltered observer\tcontact compensation torques\t' +\
	'command torques\tdesired position\tcurrent position\tcontact force\tq desired\tq current\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys used in SAI2
SIM_TIMESTAMP_KEY = "sai2::PandaApplication::simulation::timestamp";
MOM_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::momentum_based_observer";
PROCESSED_OBSERVER_KEY = "sai2::PandaApplication::controller::logging::processed_observer";
CONTACT_COMPENSATION_TORQUES_KEY = "sai2::PandaApplication::controller::logging::contact_compensation_torques";
COMMAND_TORQUES_LOGGING_KEY = "sai2::PandaApplication::controller::logging::command_torques";
DESIRED_POS_KEY = "sai2::PandaApplication::controller::logging::desired_position";
CURRENT_POS_KEY = "sai2::PandaApplication::controller::logging::current_position";
CONTACT_FORCE_KEY = "sai2::PandaApplication::simulation::current_contact_force";
DESIRED_JOINT_KEY = "sai2::PandaApplication::controller::logging::desired_joint_position";
CURRENT_JOINT_KEY = "sai2::PandaApplication::controller::logging::current_joint_position";


r_server.set(MOM_OBSERVER_LOGGING_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(PROCESSED_OBSERVER_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(CONTACT_COMPENSATION_TORQUES_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(COMMAND_TORQUES_LOGGING_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(DESIRED_POS_KEY, json.dumps([0,0,0]))
r_server.set(CURRENT_POS_KEY, json.dumps([0,0,0]))
r_server.set(CONTACT_FORCE_KEY, json.dumps([0,0,0]))
r_server.set(DESIRED_JOINT_KEY, json.dumps([0,0,0,0,0,0,0]))
r_server.set(CURRENT_JOINT_KEY, json.dumps([0,0,0,0,0,0,0]))

# data logging frequency
logger_frequency = 100.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print('Start Logging Data ... \n')

while(runloop):
	t += logger_period

	# r_mom = json.loads(r_server.get(MOM_OBSERVER_LOGGING_KEY))
	# tau_comp = json.loads(r_server.get(CONTACT_COMPENSATION_TORQUES_KEY))
	# tau_cmd = json.loads(r_server.get(COMMAND_TORQUES_LOGGING_KEY))
	# x_des = json.loads(r_server.get(DESIRED_POS_KEY))
	# x_curr = json.loads(r_server.get(CURRENT_POS_KEY))

	sim_time = json.loads(r_server.get(SIM_TIMESTAMP_KEY).decode("utf-8"))
	r_mom = json.loads(r_server.get(MOM_OBSERVER_LOGGING_KEY).decode("utf-8"))
	r_processed = json.loads(r_server.get(PROCESSED_OBSERVER_KEY).decode("utf-8"))
	tau_comp = json.loads(r_server.get(CONTACT_COMPENSATION_TORQUES_KEY).decode("utf-8"))
	tau_cmd = json.loads(r_server.get(COMMAND_TORQUES_LOGGING_KEY).decode("utf-8"))
	x_des = json.loads(r_server.get(DESIRED_POS_KEY).decode("utf-8"))
	x_curr = json.loads(r_server.get(CURRENT_POS_KEY).decode("utf-8"))
	f_contact = json.loads(r_server.get(CONTACT_FORCE_KEY).decode("utf-8"))
	q_des = json.loads(r_server.get(DESIRED_JOINT_KEY).decode("utf-8"))
	q_curr = json.loads(r_server.get(CURRENT_JOINT_KEY).decode("utf-8"))

	line = \
	str(sim_time) + '\t' + \
	" ".join([str(x) for x in r_mom]) + '\t' + \
	" ".join([str(x) for x in r_processed]) + '\t' + \
	" ".join([str(x) for x in tau_comp]) + '\t' +\
	" ".join([str(x) for x in tau_cmd]) + '\t' +\
	" ".join([str(x) for x in x_des]) + '\t' +\
	" ".join([str(x) for x in x_curr]) + '\t' +\
	" ".join([str(x) for x in f_contact]) + '\t' +\
	" ".join([str(x) for x in q_des]) + '\t' +\
	" ".join([str(x) for x in q_curr]) + '\t' +\
	'\n'

	file.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print("Elapsed time : ", elapsed_time, " seconds")
print("Loop cycles  : ", counter)
print("Frequency    : ", counter/elapsed_time, " Hz")

file.close()
