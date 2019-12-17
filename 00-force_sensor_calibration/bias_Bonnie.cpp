// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"

#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

const string bias_file_name = "../../00-force_sensor_calibration/calibration_files/Bonnie_fsensor_bias.xml";

// redis keys:
// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string JOINT_TORQUES_SENSED_KEY;
// - write
string JOINT_TORQUES_COMMANDED_KEY;

string FORCE_SENSED_KEY;

// - model
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;

void writeXml(const string file_name, const VectorXd sensor_bias);

unsigned long long controller_counter = 0;

const bool flag_simulation = false;
// const bool flag_simulation = true;

const bool inertia_regularization = true;

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

		FORCE_SENSED_KEY = "sai2::PandaApplication::sensors::force_sensor";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Bonnie::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";		

		// FORCE_SENSED_KEY = "sai2::optoforceSensor::6Dsensor::force";
		FORCE_SENSED_KEY = "sai2::ATIGamma_Sensor::Bonnie::force_torque";
	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	if(!redis_client.exists(FORCE_SENSED_KEY))
	{
		redis_client.setEigenMatrixJSON(FORCE_SENSED_KEY, VectorXd::Zero(6));
	}

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
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	
	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_otg->setMaxVelocity(M_PI/5);
	joint_task->_otg->setMaxAcceleration(M_PI);
	joint_task->_otg->setMaxJerk(3*M_PI);

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_ki = 300.0;
	joint_task->_kp = 400.0;
	joint_task->_kv = 25.0;

	// prepare successive positions
	vector<VectorXd> q_desired;
	for(int i=0 ; i<6 ; i++)
	{
		q_desired.push_back(VectorXd::Zero(dof));
	}

	VectorXd q_des_degrees = VectorXd::Zero(dof);

	q_des_degrees << 0, 30, 90, -90, -30, 90, 0;
	q_desired[0] = M_PI/180.0 * q_des_degrees; 

	q_des_degrees << 0, 30, 90, -90, 60, 90, -135;
	q_desired[1] = M_PI/180.0 * q_des_degrees; 

	q_des_degrees << 0, 30, 90, -90, 60, 90, -45;
	q_desired[2] = M_PI/180.0 * q_des_degrees; 

	q_des_degrees << 0, 30, 90, -90, 60, 90, 45;
	q_desired[3] = M_PI/180.0 * q_des_degrees; 

	q_des_degrees << 0, 30, 90, -90, 60, 90, 135;
	q_desired[4] = M_PI/180.0 * q_des_degrees; 

	q_des_degrees << 0, 30, 90, -90, 150, 90, 0;
	q_desired[5] = M_PI/180.0 * q_des_degrees; 

	int measurement_number = 0;
	joint_task->_desired_position = q_desired[measurement_number];
	const int measurement_total_length = 1000;
	int measurement_counter = measurement_total_length;

	// force readings
	VectorXd bias_force = VectorXd::Zero(6);
	VectorXd current_force_measurement = VectorXd::Zero(6);
	VectorXd current_force_sensed = VectorXd::Zero(6);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		current_force_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEY);

		// update model
		if(flag_simulation)
		{
			robot->updateModel();
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();
		}
		joint_task->updateTaskModel(N_prec);

		// compute torques
		joint_task->computeTorques(joint_task_torques);

		command_torques = joint_task_torques;

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// cout << (joint_task->_current_position - joint_task->_desired_position).norm() << endl;

		if((joint_task->_current_position - joint_task->_desired_position).norm() < 0.015)
		{
			measurement_counter--;

			if(measurement_counter > measurement_total_length/4.0 && measurement_counter < measurement_total_length/4.0*3.0)
			{
				current_force_measurement += current_force_sensed;
			}

			if(measurement_counter == 0)
			{
				bias_force += current_force_measurement*2.0/measurement_total_length;
				current_force_measurement.setZero();

				measurement_number++;
				if(measurement_number < 6)
				{
					joint_task->_desired_position = q_desired[measurement_number];
					measurement_counter = measurement_total_length;
				}
				else
				{
					cout << "bias calibration finished" << endl;
					runloop = false;
				}
			}

		}

		controller_counter++;

	}

	bias_force = bias_force/6.0;
	writeXml(bias_file_name, bias_force);
	cout << "force bias :\n" << bias_force.transpose() << endl << endl;

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
    cout << "\n";
    cout << "Controller Loop run time  : " << end_time << " seconds\n";
    cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}

void writeXml(const string file_name, const VectorXd sensor_bias)
{
	if(sensor_bias.size() != 6)
	{
		cout << "bias should be a vector of length 6\nXml file not written" << endl;
		return;
	}

	cout << "write bias to file " << file_name << endl;

	ofstream file;
	file.open(file_name);

	if(file.is_open())
	{
		file << "<?xml version=\"1.0\" ?>\n";
		file << "<force_bias value=\"" << sensor_bias.transpose() << "\"/>\n";
		file.close();
	}
	else
	{
		cout << "could not create xml file" << endl;
	}
}
