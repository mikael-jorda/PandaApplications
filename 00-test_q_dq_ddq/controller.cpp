// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// logger
string LOG_Q_KEY = "sai2::PandaApplication::logger:q";
string LOG_DQ_DRIVER_KEY = "sai2::PandaApplication::logger:dq_driver";
string LOG_DQ_DIFF_KEY = "sai2::PandaApplication::logger:dq_diff";
string LOG_DDQ_KEY = "sai2::PandaApplication::logger:ddq";
string LOG_DDQ_DIFF_KEY = "sai2::PandaApplication::logger:ddq_diff";

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
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Bonnie::sensors::torques";
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
	int dof = robot->dof();
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	VectorXd q = VectorXd::Zero(dof);
	VectorXd q_prev = VectorXd::Zero(dof);
	VectorXd dq_driver = VectorXd::Zero(dof);
	VectorXd dq_driver_prev = VectorXd::Zero(dof);
	VectorXd dq_diff = VectorXd::Zero(dof);
	VectorXd dq_diff_prev = VectorXd::Zero(dof);
	VectorXd ddq = VectorXd::Zero(dof);
	VectorXd ddq_diff = VectorXd::Zero(dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	double prev_time = 0;
	double time = 0;
	bool fTimerDidSleep = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		prev_time = time;
		time = timer.elapsedTime() - start_time;
		double dt = time - prev_time;

		// read robot state from redis
		q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		dq_driver = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		dq_diff = (q - q_prev)/dt;
		ddq = (dq_driver - dq_driver_prev)/dt;
		ddq_diff = (dq_diff - dq_diff_prev)/dt;

		q_prev = q;
		dq_diff_prev = dq_diff;
		dq_driver_prev = dq_driver;

		redis_client.setEigenMatrixJSON(LOG_Q_KEY, q);
		redis_client.setEigenMatrixJSON(LOG_DQ_DRIVER_KEY, dq_driver);
		redis_client.setEigenMatrixJSON(LOG_DQ_DIFF_KEY, dq_diff);
		redis_client.setEigenMatrixJSON(LOG_DDQ_KEY, ddq);
		redis_client.setEigenMatrixJSON(LOG_DDQ_DIFF_KEY, ddq_diff);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
