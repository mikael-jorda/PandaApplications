// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

const string CYLINDER_KEY = "tmp::cylinder";

const string robot_file = "../resources/00-test_jacobians/panda_arm.urdf";

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->updateKinematics();
	int dof = robot->dof();

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

	int n_supppl_joint_config = 50;
	for(int i=0; i<n_supppl_joint_config ; i++)
	{
		q_test = VectorXd::Random(dof)*M_PI;
		joint_configurations.push_back(q_test);
	}

	string contact_link = "link5";
	Vector3d cylinder_base = Vector3d(0,0,-0.26);
	Vector3d weird_offset = Vector3d(0, 0.07, 0);
	double theta = -0.25;
	Vector3d cylinder_x = Vector3d(1, 0, 0);
	Vector3d cylinder_y = Vector3d(0, cos(theta), sin(theta));
	Vector3d cylinder_z = Vector3d(0, -sin(theta), cos(theta));
	Matrix3d R_cylinder = Matrix3d::Zero();
	R_cylinder << cylinder_x, cylinder_y, cylinder_z;
	// cout << R_cylinder << endl;
	double cylinder_length = 0.3;
	double cylinder_radius = 0.06;

	int n_points_length = 20;
	int n_points_circle = 20;

	Eigen::MatrixXd sample_points = MatrixXd::Zero(3,n_points_circle * n_points_length);
	Eigen::MatrixXd circle = MatrixXd::Zero(2, n_points_circle);

	for(int i=0 ; i<n_points_circle; i++)
	{
		double alpha = 2*M_PI*i/n_points_circle;
		circle(0,i) = cylinder_radius*cos(alpha);
		circle(1,i) = cylinder_radius*sin(alpha);
	}

	for(int i=0; i<n_points_length ; i++)
	{
		sample_points.block(0, n_points_circle*i, 2, n_points_circle) = circle;
		sample_points.block(2, n_points_circle*i, 1, n_points_circle) = i*cylinder_length/n_points_length*VectorXd::Ones(n_points_circle).transpose();
	}
	for(int i=0 ; i<n_points_circle*n_points_length ; i++)
	{
		sample_points.col(i) += cylinder_base;
	}
	sample_points = R_cylinder*sample_points;
	for(int i=0 ; i<n_points_circle*n_points_length ; i++)
	{
		sample_points.col(i) += weird_offset;
	}

	redis_client.setEigenMatrixJSON(CYLINDER_KEY, sample_points);

	MatrixXd J = MatrixXd::Zero(3,dof);

	for(int try_number=0; try_number < joint_configurations.size() ; try_number++)
	{
		robot->_q = joint_configurations[try_number];
		robot->updateKinematics();

		double max_max_svd = 0;
		double min_max_svd = 100;
		double max_min_svd = 0;
		double min_min_svd = 100;
		for(int i=0 ; i<n_points_circle*n_points_length ; i++)
		{
			robot->Jv(J, contact_link, sample_points.col(i));
			JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
			Vector3d sigma = svd.singularValues();
			if(sigma(0) > max_max_svd)
			{
				max_max_svd = sigma(0);
			}
			if(sigma(0) < min_max_svd)
			{
				min_max_svd = sigma(0);
			}
			if(sigma(2) > 1e-2 && sigma(2) < min_min_svd)
			{
				min_min_svd = sigma(2);
			}
			if(sigma(1) < min_min_svd)
			{
				min_min_svd = sigma(1);
			}
			if(sigma(2) > max_min_svd)
			{
				max_min_svd = sigma(2);
			}
			// cout << svd.singularValues().transpose() << endl;
		}
		cout << "------------------------------" << endl;
		cout << max_max_svd << "\t" << min_max_svd << endl;
		cout << max_min_svd << "\t" << min_min_svd << endl;
		cout << endl;
	}

	robot->Jv(J, contact_link, Vector3d::Zero());
	JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
	// cout << svd.singularValues().transpose() << endl;
	cout << svd.singularValues().transpose() << endl;

	return 0;
}
