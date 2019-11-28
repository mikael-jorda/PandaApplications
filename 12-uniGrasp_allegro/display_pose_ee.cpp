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

const vector<string> robot_files = {
	"./resources/panda_arm.urdf",
	"./resources/panda_arm.urdf",
};

const int n_robots = robot_files.size();

const string filename = "joint_positions.txt";


// redis keys:
// - read:
vector<string> JOINT_ANGLES_KEYS  = {
	"sai2::WarehouseSimulation::panda1::sensors::q",
	"sai2::WarehouseSimulation::panda2::sensors::q",
};
vector<string> JOINT_VELOCITIES_KEYS = {
	"sai2::WarehouseSimulation::panda1::sensors::dq",
	"sai2::WarehouseSimulation::panda2::sensors::dq",
};
vector<string> FORCE_SENSED_KEYS = {
	"sai2::WarehouseSimulation::panda1::sensors::f_op",
	"sai2::WarehouseSimulation::panda2::sensors::f_op",
};

const string REMOTE_ENABLED_KEY = "sai2::WarehouseSimulation::sensors::remote_enabled";
const string RESTART_CYCLE_KEY = "sai2::WarehouseSimulation::sensors::restart_cycle";

// - write
vector<string> JOINT_TORQUES_COMMANDED_KEYS = {
	"sai2::WarehouseSimulation::panda1::actuators::fgc",
	"sai2::WarehouseSimulation::panda2::actuators::fgc",
};

// - model
vector<string> MASSMATRIX_KEYS = 
{
	"sai2::WarehouseSimulation::panda1::model::mass_matrix",
	"sai2::WarehouseSimulation::panda2::model::mass_matrix",
};

vector<string> CORIOLIS_KEYS = 
{
	"sai2::WarehouseSimulation::panda1::model::coriolis",
	"sai2::WarehouseSimulation::panda2::model::coriolis",	
};

vector<string> ROBOT_GRAVITY_KEYS = 
{
	"sai2::WarehouseSimulation::panda1::model::gravity",
	"sai2::WarehouseSimulation::panda2::model::gravity",		
};

const bool flag_simulation = false;
// const bool flag_simulation = true;

int main() {

	if(!flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEYS[0] = "sai2::FrankaPanda::Clyde::actuators::fgc";
		JOINT_ANGLES_KEYS[0]  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEYS[0] = "sai2::FrankaPanda::Clyde::sensors::dq";
		MASSMATRIX_KEYS[0] = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEYS[0] = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEYS[0] = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";		
		FORCE_SENSED_KEYS[0] = "sai2::optoforceSensor::6Dsensor::force";

		JOINT_TORQUES_COMMANDED_KEYS[1] = "sai2::FrankaPanda::Bonnie::actuators::fgc";
		JOINT_ANGLES_KEYS[1]  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::dq";
		MASSMATRIX_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_GRAVITY_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";	
	}


	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// prepare robot task controllers
	vector<int> dof;

	// load robots
	vector<Sai2Model::Sai2Model*> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(new Sai2Model::Sai2Model(robot_files[i], false));
		robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
		robots[i]->updateModel();
	}

	const string link_name_1 = "link7";
	const string link_name_2 = "link7";
	const Vector3d pos_in_link_1 = Vector3d(0,0,0.107);
	const Vector3d pos_in_link_2 = Vector3d(0,0,0.146);

	Affine3d ee_pose_1 = Affine3d::Identity();
	Affine3d ee_pose_2 = Affine3d::Identity();

	// read robot state from redis
	for(int i=0 ; i<n_robots ; i++)
	{
		robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
	}

	robots[0]->transform(ee_pose_1, link_name_1, pos_in_link_1);
	robots[1]->transform(ee_pose_2, link_name_2, pos_in_link_2);

	cout << endl;
	cout << "position Clyde :\n" << ee_pose_1.translation().transpose() << endl;
	cout << "orientation clyde :\n" << ee_pose_1.linear() << endl;
	cout << "joint configuration clyde :\n" << robots[0]->_q.transpose() << endl;
	cout << endl;

	cout << endl;
	cout << "position Bonnie :\n" << ee_pose_2.translation().transpose() << endl;
	cout << "orientation Bonnie :\n" << ee_pose_2.linear() << endl;
	cout << "joint configuration Bonnie :\n" << robots[1]->_q.transpose() << endl;
	cout << endl;

	Affine3d ee_Clyde_to_ee_Bonnie = Affine3d::Identity();
	ee_Clyde_to_ee_Bonnie.linear() << 1, 0, 0,
									0, -1, 0,
									0, 0, -1;

	Affine3d T_Clyde_to_Bonnie = Affine3d::Identity();

	T_Clyde_to_Bonnie = ee_pose_1 * ee_Clyde_to_ee_Bonnie * ee_pose_2.inverse();
	cout << "pos Bonnie in Cyde frame :\n" << T_Clyde_to_Bonnie.translation().transpose() << endl;
	cout << "rot Bonnie in clyde frame :\n" << T_Clyde_to_Bonnie.linear() << endl << endl;

	AngleAxisd aa_rot_C_to_B = AngleAxisd(T_Clyde_to_Bonnie.linear());

	cout << "Axis : " << aa_rot_C_to_B.axis().transpose() << endl;
	cout << " Angle : " << aa_rot_C_to_B.angle() << endl; 


	/*
	 * Expected values :
	 * position :   0.273298     1.08895 -0.00602478
	 * rotation :
	 * 0.174794   0.984093  0.0317545
	   -0.9845   0.175155 -0.0089491
	-0.0143687 -0.0296981   0.999456

	 * Axis : -0.0105365  0.0234218   -0.99967
 	 * Angle : 1.39519


	*/


	// ofstream myfile;
	// myfile.open(filename);
	// myfile << "Clyde first, Bonnie second" << endl;
	// myfile << robots[0]->_q.transpose() << endl;
	// myfile << robots[1]->_q.transpose() << endl;
	// myfile.close();

	return 0;
}
