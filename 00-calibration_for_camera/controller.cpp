// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"

#include <iostream>
#include <string>
#include <fstream>

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
std::string JOINT_ANGLES_KEY = "sai2::FrankaPanda::Bonnie::sensors::q";;
std::string JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";;

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool inertia_regularization = true;

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

	VectorXd command_torques = VectorXd::Zero(robot->dof());
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// set up data recording
	const string marker_link = "link7";
	Affine3d T_base_marker_link = Affine3d::Identity();

	ofstream data_file;
	data_file.open("marker_link_pose_in_robot_frame.txt");

	while (runloop) {

		// wait for user input
		char s = getchar();
		if(s == 'q')
		{
			runloop = false;
			break;
		}

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateKinematics();

		// get position of marker_link and write to file
		robot->transform(T_base_marker_link, marker_link);
		data_file << T_base_marker_link.linear().col(0).transpose() << " " << T_base_marker_link.linear().col(1).transpose() << " " << T_base_marker_link.linear().col(2).transpose() << " " << T_base_marker_link.translation().transpose() << "\n";


	}

	data_file.close();
	// send to redis
	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	return 0;
}
