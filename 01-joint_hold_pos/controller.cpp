// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "../resources/01-joint_hold_pos/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
// - write
const std::string TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

// - gripper
const std::string GRIPPER_MODE_KEY  = "sai2::PandaApplication::gripper::mode"; // m for move and g for graps
const std::string GRIPPER_MAX_WIDTH_KEY  = "sai2::PandaApplication::gripper::max_width";
const std::string GRIPPER_CURRENT_WIDTH_KEY  = "sai2::PandaApplication::gripper::current_width";
const std::string GRIPPER_DESIRED_WIDTH_KEY  = "sai2::PandaApplication::gripper::desired_width";
const std::string GRIPPER_DESIRED_SPEED_KEY  = "sai2::PandaApplication::gripper::desired_speed";
const std::string GRIPPER_DESIRED_FORCE_KEY  = "sai2::PandaApplication::gripper::desired_force";

unsigned long long controller_counter = 0;

int main() {
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
	auto joint_task = new Sai2Primitives::JointTask(robot);

	MatrixXd N_prec = MatrixXd::Identity(robot->dof(), robot->dof());
	VectorXd joint_task_torques = VectorXd::Zero(robot->dof());

	VectorXd command_torques = VectorXd::Zero(robot->dof());

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

		// update model
		robot->updateModel();
		joint_task->updateTaskModel(N_prec);

		// compute torques
		// joint_task->_desired_position(2) = initial_q(2) + 10/180.0*M_PI * sin(2*M_PI*0.1*time);
		joint_task->computeTorques(joint_task_torques);

		command_torques = joint_task_torques;

		// send to redis
		redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
