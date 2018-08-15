// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

const string robot_file = "../resources/00-test_jacobians/panda_arm.urdf";

void displayInfo(Sai2Model::Sai2Model* robot, VectorXd q, string ee_link, Vector3d ee_point)
{

	MatrixXd J = MatrixXd::Zero(6,7);
	Affine3d T;

	robot->_q = q;
	robot->updateModel();

	cout << "joint config : " << robot->_q.transpose() << endl << endl;
	robot->transform(T, ee_link, ee_point);
	robot->J_0(J, ee_link, ee_point);
	cout << "position of end effector point : " << T.translation().transpose() << endl << endl;
	cout << "rotation of end effector frame :\n" << T.linear() << endl << endl;
	cout << "Jacobian at end effector :\n" << J << endl << endl;
	cout << endl << endl;	
}

int main() {

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

	std::vector<VectorXd> joint_configurations;
	VectorXd q_test(7);

	q_test << 0, 0, 0, 0, 0, 0, 0;
	q_test = q_test * (M_PI/180.0);
	joint_configurations.push_back(q_test);

	q_test << 0, -100, 0, -180, 0, 90, 0;
	q_test = q_test * (M_PI/180.0);
	joint_configurations.push_back(q_test);

	q_test << 25, -100, -55, -180, 35, 90, 15;
	q_test = q_test * (M_PI/180.0);
	joint_configurations.push_back(q_test);

	string ee_link = "link7";
	Vector3d ee_point = Vector3d(0.02, -0.04, 0.1);



	cout << endl << endl;
	for(int i=0 ; i<3 ; i++)
	{
		displayInfo(robot, joint_configurations[i], ee_link, ee_point);
		cout << "---------------------------------------------\n" << endl;
	}
	cout << endl << endl;


	return 0;
}
