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


using namespace std;
using namespace Eigen;

const vector<string> robot_files = {
	"./resources/iiwa7.urdf",
	"./resources/panda_arm.urdf",
};

const int n_robots = robot_files.size();


int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// load robots
	auto robot_kuka = new Sai2Model::Sai2Model(robot_files[0], false);
	auto robot_Bonnie = new Sai2Model::Sai2Model(robot_files[1], false);

	robot_kuka->_q << -0.0188136, 1.54763, -2.55237, -0.169772, 0.596577, 0.446634, -0.298062;
	robot_Bonnie->_q << 1.08714, -1.46005, 1.5774, -0.435793, 1.22249, 2.70925, -0.805811;

	robot_kuka->updateModel();
	robot_Bonnie->updateModel();

	const string link_name_kuka = "link6";
	const string link_name_Bonnie = "link7";

	double kuka_flange_length = 0.038;
	double optoforce_length = 0.035;
	double panda_link_to_ee = 0.107;
	double ATI_sensot_and_spacer_length = 0.039;

	const Vector3d pos_in_link_kuka = Vector3d(0,0, kuka_flange_length + optoforce_length);
	const Vector3d pos_in_link_Bonnie = Vector3d(0,0, panda_link_to_ee + ATI_sensot_and_spacer_length);

	Affine3d ee_pose_kuka = Affine3d::Identity();
	Affine3d ee_pose_Bonnie = Affine3d::Identity();

	robot_kuka->transform(ee_pose_kuka, link_name_kuka, pos_in_link_kuka);
	robot_Bonnie->transform(ee_pose_Bonnie, link_name_Bonnie, pos_in_link_Bonnie);

	cout << endl;
	cout << "position kuka :\n" << ee_pose_kuka.translation().transpose() << endl;
	cout << "orientation kuka :\n" << ee_pose_kuka.linear() << endl;
	cout << "joint configuration kuka :\n" << robot_kuka->_q.transpose() << endl;
	cout << endl;

	cout << endl;
	cout << "position Bonnie :\n" << ee_pose_Bonnie.translation().transpose() << endl;
	cout << "orientation Bonnie :\n" << ee_pose_Bonnie.linear() << endl;
	cout << "joint configuration Bonnie :\n" << robot_Bonnie->_q.transpose() << endl;
	cout << endl;

	Affine3d ee_kuka_to_ee_Bonnie = Affine3d::Identity();
	ee_kuka_to_ee_Bonnie.linear() << 1, 0, 0,
									0, -1, 0,
									0, 0, -1;

	Affine3d T_kuka_to_Bonnie = Affine3d::Identity();

	T_kuka_to_Bonnie = ee_pose_kuka * ee_kuka_to_ee_Bonnie * ee_pose_Bonnie.inverse();
	cout << "pos Bonnie in kuka frame :\n" << T_kuka_to_Bonnie.translation().transpose() << endl;
	cout << "rot Bonnie in kuka frame :\n" << T_kuka_to_Bonnie.linear() << endl << endl;

	AngleAxisd aa_rot_K_to_B = AngleAxisd(T_kuka_to_Bonnie.linear());

	cout << "Axis : " << aa_rot_K_to_B.axis().transpose() << endl;
	cout << " Angle : " << aa_rot_K_to_B.angle() << endl; 
	cout << " Angle in degrees : " << aa_rot_K_to_B.angle()/M_PI*180.0 << endl; 


	/*
	 * Expected values :
	 * position :    1.73603  -0.421108 -0.0130478
	 * rotation :
	 *  0.450522   0.890417  0.0647095
		-0.891364   0.444572  0.0884651
		0.0500028 -0.0975352   0.993975


	 * Axis : -0.103822 0.00820904  -0.994562
 	 * Angle : 1.11014
 	 * Angle in degrees : 63.6064 
 	 */

	return 0;
}
