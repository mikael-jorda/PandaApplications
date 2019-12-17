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

// redis keys:
// - read:
string JOINT_ANGLES_KEY = "sai2::FrankaPanda::Clyde::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
// - write
string JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";

string R_PALM_KEY = "sai2::allegroHand::controller::palm_orientation";

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

	// prepare controller : zero torques
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	
	// hand palm orientation
	Matrix3d R_ee_palm = Matrix3d::Identity();
	Matrix3d R_world_palm = Matrix3d::Identity();

	R_ee_palm << 0, 0, 1,
				0, -1, 0,
				1, 0, 0;

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

		robot->updateModel();

		// compute palm rotation and send to hand
		Matrix3d R_world_ee = Matrix3d::Identity();
		robot->rotation(R_world_ee, "link7");
		R_world_palm = R_world_ee * R_ee_palm;
		redis_client.setEigenMatrixJSON(R_PALM_KEY, R_world_palm);

		// send zero torques to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}

	double end_time = timer.elapsedTime();
    cout << "\n";
    cout << "Controller Loop run time  : " << end_time << " seconds\n";
    cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
