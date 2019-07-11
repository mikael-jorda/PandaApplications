// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "filters/ButterworthFilter.h"

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

const string filename = "ee_pose.txt";

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

const bool flag_simulation = false;
// const bool flag_simulation = true;

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
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Clyde::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";		

		GRIPPER_MODE_KEY  = "sai2::FrankaPanda::Clyde::gripper::mode"; // m for move and g for graps
		GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::Clyde::gripper::max_width";
		GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::Clyde::gripper::current_width";
		GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::Clyde::gripper::desired_width";
		GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::Clyde::gripper::desired_speed";
		GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::Clyde::gripper::desired_force";
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

	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0,0,0.167);

	Affine3d ee_pose = Affine3d::Identity();

	// read robot state from redis
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

	robot->transform(ee_pose, link_name, pos_in_link);

	cout << endl;
	cout << "position :\n" << ee_pose.translation().transpose() << endl;
	cout << "orientation :\n" << ee_pose.linear() << endl;
	cout << "joint configuration :\n" << robot->_q.transpose() << endl;
	cout << endl;

	ofstream myfile;
	myfile.open(filename);
	myfile << ee_pose.linear().transpose() << endl;
	myfile << ee_pose.translation().transpose() << endl;
	myfile.close();

	return 0;
}
