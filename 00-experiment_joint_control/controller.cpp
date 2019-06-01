// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "filters/ButterworthFilter.h"
#include "tasks/JointTask.h"

#include <iostream>
#include <string>
#include <fstream>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "resources/panda_arm.urdf";
const string robot_name = "PANDA";

#define DYNAMIC_DECOUPLING       0      
#define IMPEDANCE                1      
#define INERTIA_SATURATION       2      
#define INERTIA_REGULARIZATION   3      

#define SLOW_SPEED    0
#define MEDIUM_SPEED  1
#define HIGH_SPEED    2

#define LOW_GAINS        0
#define MEDIUM_GAINS     1
#define HIGH_GAINS       2

#define DEBUG                      0
#define EXPERIMENT_TRACKING        1
#define EXPERIMENT_REGULATION      2
#define GO_TO_INITIAL              3

int state = EXPERIMENT_TRACKING;
int controller_type = 0;
int speed = 0;
int gains_value = 0;

int experiment_number = 1;
int number_of_oscillation_per_experiment = 3;
Vector3d experiment_speed_nominal_period = Vector3d(10.0, 5.0, 1.0);
int experiment_countdown = 1000 * number_of_oscillation_per_experiment * experiment_speed_nominal_period(0);


// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

// - gripper
std::string GRIPPER_MODE_KEY; // m for move and g for graps
std::string GRIPPER_MAX_WIDTH_KEY;
std::string GRIPPER_CURRENT_WIDTH_KEY;
std::string GRIPPER_DESIRED_WIDTH_KEY;
std::string GRIPPER_DESIRED_SPEED_KEY;
std::string GRIPPER_DESIRED_FORCE_KEY;

// gains
const string KP_JOINT_KEY = "sai2::PandaApplication::controller:kp_joint";
const string KV_JOINT_KEY = "sai2::PandaApplication::controller:kv_joint";

const string DESIRED_POSITION_KEY = "sai2::PandaApplication::controller::q_desired";

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const string prefix_path = "../../00-experiment_joint_control/data_files/";
string create_filename();

void write_first_line(ofstream& file_handler);

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

		GRIPPER_MODE_KEY  = "sai2::PandaApplication::gripper::mode"; // m for move and g for graps
		GRIPPER_MAX_WIDTH_KEY  = "sai2::PandaApplication::gripper::max_width";
		GRIPPER_CURRENT_WIDTH_KEY  = "sai2::PandaApplication::gripper::current_width";
		GRIPPER_DESIRED_WIDTH_KEY  = "sai2::PandaApplication::gripper::desired_width";
		GRIPPER_DESIRED_SPEED_KEY  = "sai2::PandaApplication::gripper::desired_speed";
		GRIPPER_DESIRED_FORCE_KEY  = "sai2::PandaApplication::gripper::desired_force";		
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";		

		GRIPPER_MODE_KEY  = "sai2::FrankaPanda::gripper::mode"; // m for move and g for graps
		GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::gripper::max_width";
		GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::gripper::current_width";
		GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::gripper::desired_width";
		GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::gripper::desired_speed";
		GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::gripper::desired_force";
	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller	
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = false;

	redis_client.setEigenMatrixJSON(DESIRED_POSITION_KEY, robot->_q);

	double oscillation_period = 10.0;
	double oscillation_amplitude = M_PI/12;

	double initial_time = 0;

	VectorXd frequency_scaling = VectorXd::Zero(dof);
	frequency_scaling << 0.6, 0.8, 0.9, 1.0, 1.1, 1.2, 1.4;

	// prepare file to write data
	bool newfile = false;
	string filename = create_filename();
	ofstream data_file;
	data_file.open(filename);
	if(!data_file.is_open())
	{
		cout << "could not open file at " << filename << endl;
		return 0;
	}
	write_first_line(data_file);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;
		dt = current_time - prev_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			robot->_M_inv = robot->_M.inverse();

			coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
		}

		N_prec.setIdentity();
		joint_task->updateTaskModel(N_prec);

		// set gains
		if(gains_value == LOW_GAINS)
		{
			joint_task->_kp = 100.0;
			joint_task->_kv = 20.0;
		}
		else if(gains_value == MEDIUM_GAINS)
		{
			joint_task->_kp = 200.0;
			joint_task->_kv = 25.0;			
		}
		else if(gains_value == MEDIUM_GAINS)
		{
			joint_task->_kp = 400.0;
			joint_task->_kv = 35.0;			
		}
		redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
		redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));

		// set experiment type
		if(state == DEBUG)
		{
			joint_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_POSITION_KEY);
			joint_task->_kp = stod(redis_client.get(KP_JOINT_KEY));
			joint_task->_kv = stod(redis_client.get(KV_JOINT_KEY));
		}
		else if (state == EXPERIMENT_REGULATION)
		{
			// read force sensor
		}
		else if(state == EXPERIMENT_TRACKING)
		{

			if(newfile)
			{
				data_file.open(create_filename());
				write_first_line(data_file);
				newfile = false;
			}

			data_file << joint_task->_desired_position.transpose() << '\t' << robot->_q.transpose() << '\t' << 
				robot->_dq.transpose() << '\t' << command_torques.transpose() << endl;

			for(int i=0 ; i<dof ; i++)
			{
				joint_task->_desired_position(i) = initial_q(i) + 
					oscillation_amplitude*sin(2*M_PI/experiment_speed_nominal_period(speed)*frequency_scaling(i)*(current_time - initial_time));
			}

			// switch scenario
			if(experiment_countdown == 0)
			{

				data_file.close();
				// initial_time = current_time;
				// initial_q = robot->_q;
				
				experiment_countdown = 1000 * number_of_oscillation_per_experiment * experiment_speed_nominal_period(speed);

				cout << "finished experiment : " << experiment_number << "/36" << endl;
				cout << "speed : " << speed << "\tcontroller type : " << controller_type << "\tgains : " << gains_value << endl;
				cout << endl;
				newfile = true;
				experiment_number++;
				gains_value++;
				if(gains_value == 3)
				{
					gains_value = 0;
					controller_type++;
					if(controller_type == 4)
					{
						controller_type = 0;
						speed++;

						state = GO_TO_INITIAL;
						joint_task->_use_interpolation_flag = true;
						joint_task->_desired_position = initial_q; 
						joint_task->_ki = 5.0;
					}
				}
			}
			experiment_countdown--;
		}
		else if (state == GO_TO_INITIAL)
		{
			// don't update desired position
			// cout << "joint error : " << (joint_task->_desired_position - joint_task->_current_position).norm() << endl;
			if((joint_task->_desired_position - joint_task->_current_position).norm() < 0.15)
			{
				if(speed < 3)
				{
					joint_task->_ki = 0;
					joint_task->_use_interpolation_flag = false;
					initial_time = current_time;
					state = EXPERIMENT_TRACKING;
				}
			}
		}

		// set controller type
		if(controller_type == DYNAMIC_DECOUPLING)
		{
			joint_task->computeTorques(joint_task_torques);
		}
		else if(controller_type == IMPEDANCE)
		{
			robot->_M.setIdentity();
			joint_task->computeTorques(joint_task_torques);
		}
		else if(controller_type == INERTIA_SATURATION)
		{
			for(int i=0 ; i<dof ; i++)
			{
				if(robot->_M(i,i) < 0.1)
				{
					robot->_M(i,i) = 0.1;
				}
			}
			joint_task->computeTorques(joint_task_torques);
		}
		else if(controller_type == INERTIA_REGULARIZATION)
		{
			for(int i=0 ; i<dof ; i++)
			{
				robot->_M(i,i) += 0.1;
			}
			joint_task->computeTorques(joint_task_torques);
		}
		else
		{
			command_torques.setZero(dof);
		}

		// send to redis
		command_torques = joint_task_torques + coriolis;

		// command_torques.setZero(dof);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// write file

		prev_time = current_time;
		controller_counter++;
	}

	command_torques.setZero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	data_file.close();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

string create_filename()
{
	string return_filename = prefix_path;

	switch(state)
	{
		case EXPERIMENT_REGULATION :
			return_filename += "regulation_";
			break;
		case EXPERIMENT_TRACKING :
			return_filename += "tracking_";
			break;
		case DEBUG :
			return_filename += "debug_";
			break;
		default :
			return_filename += "should_not_happen_";
			break;
	}

	switch(speed)
	{
		case SLOW_SPEED :
			return_filename += "slow_";
			break;
		case MEDIUM_SPEED :
			return_filename += "medium_";
			break;
		case HIGH_SPEED :
			return_filename += "fast_";
			break;
	}

	switch(controller_type)
	{
		case DYNAMIC_DECOUPLING :
			return_filename += "dynamicDecoupling_";
			break;
		case IMPEDANCE :
			return_filename += "impedance_";
			break;
		case INERTIA_REGULARIZATION :
			return_filename += "inertiaRegularization_";
			break;
		case INERTIA_SATURATION :
			return_filename += "inertiaSaturation_";
			break;
	}

	switch(gains_value)
	{
		case LOW_GAINS :
			return_filename += "lowGains";
			break;
		case MEDIUM_GAINS :
			return_filename += "mediumGains";
			break;
		case HIGH_SPEED :
			return_filename += "highGains";
			break;
	}

	return_filename += ".txt";

	return return_filename;

}

void write_first_line(ofstream& file_handler)
{

	switch(state)
	{
		case EXPERIMENT_REGULATION :
			file_handler << "desired_position[7]\tcurrent_position[7]\tcurrent_velocity[7]\tcommand_torques[7]\tsensed_force[6]\n";
			break;
		case EXPERIMENT_TRACKING :
			file_handler << "desired_position[7]\tcurrent_position[7]\tcurrent_velocity[7]\tcommand_torques[7]\n";
			break;
		case DEBUG :
			file_handler << "debug_file\n";
			break;
	}

}
