// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "tasks/TwoHandTwoRobotsTask.h"

#include <iostream>
#include <string>
#include <stdexcept>

#include "safe_ptr.h"

#include <signal.h>
bool runloop = false;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const vector<string> robot_files = {
	"./resources/panda_arm_flat_ee.urdf",
	"./resources/panda_arm_flat_ee.urdf",
};
const vector<string> robot_names = {
	"PANDA1",
	"PANDA2",
};

MatrixXd computeGInverseAtGeometricCenterExplicit(const Matrix3d Rg, 
		const Vector3d contact_location_1,
		const Vector3d contact_location_2);

const int n_robots = robot_names.size();

int main() {


	// MatrixXd _grasp_matrix;
	// Matrix3d _R_grasp_matrix;
	// Vector3d geometric_center = Vector3d(0,0,0);
	// vector<Vector3d> _contact_locations;
	// _contact_locations.push_back(Vector3d(0, 0, 0));
	// _contact_locations.push_back(Vector3d(1, 0, 0));
	// vector<int> _contact_constrained_rotations(2,2);


	// Sai2Model::graspMatrixAtGeometricCenter(_grasp_matrix, _R_grasp_matrix, geometric_center,
	// 		_contact_locations, _contact_constrained_rotations);

	// Vector3d xl = _R_grasp_matrix.col(0);
	// Matrix3d xl_cross = Sai2Model::CrossProductOperator(xl);
	// double l = (_contact_locations[1] - _contact_locations[0]).norm();


	// cout << _grasp_matrix << endl << endl;
	// cout << _grasp_matrix.inverse() << endl << endl;
	// // cout << _grasp_matrix.transpose().inverse() << endl;

	// cout << xl_cross/l << endl << endl;
	// cout << (Matrix3d::Identity() + xl_cross*xl_cross)/2.0 << endl << endl;

	// return 0;

	// position of robots in world
	vector<Affine3d> robot_pose_in_world;
	Affine3d pose = Affine3d::Identity();
	pose.translation() = Vector3d(0, -0.5, 0.0);
	// pose.linear() = AngleAxisd(0.3010693, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	pose.translation() = Vector3d(0, 0.5, 0.0);
	// pose.translation() = Vector3d(-0.06, 0.57, 0.0);
	// pose.linear() = AngleAxisd(-1.0864675, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	// load robots
	vector<sf::safe_ptr<Sai2Model::Sai2Model>> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(sf::safe_ptr<Sai2Model::Sai2Model>(robot_files[i], false, robot_pose_in_world[i]));
	}

	robots[0]->_q << 0,0.5,0.45,-0.2,0,0.1,0.98;
	robots[1]->_q << 0,0.5,0,0.2,0,0.1,0;

	robots[0]->updateModel();
	robots[1]->updateModel();

	// contacts locations
	const string link_name = "link7";
	// const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.125);
	const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.115);
	vector<Vector3d> contact_locations(2,Vector3d::Zero());
	vector<int> constrained_rotations(2, 2);

	for(int i=0 ; i<n_robots ; i++)
	{
		robots[i]->positionInWorld(contact_locations[i], link_name, pos_in_link);
	}

	// cout << "contact locations :\n" << contact_locations[0].transpose() << endl << contact_locations[1].transpose() << endl;
	// object frame location
	MatrixXd G = MatrixXd::Zero(12,12);
	Vector3d geometric_center = Vector3d::Zero();
	Matrix3d R_g = Matrix3d::Identity();
	MatrixXd G_inv;

	double G_computation_time = 0;	
	double Ginv_computation_time = 0;	
	double Ginv_explicit_computation_time = 0;	
	double Glin_solve_computation_time = 0;	
	double max_error = 0;

	auto start = chrono::high_resolution_clock::now();
	double duration = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();

	double n_iterations = 10000.0;
	for(int k=0 ; k<n_iterations ; k++)
	{
		// random joint angles
		robots[0]->_q = VectorXd::Random(7);
		robots[1]->_q = VectorXd::Random(7);
		robots[0]->updateModel();
		robots[1]->updateModel();

		cout << robots[0]->_q.transpose() << endl;

		start = chrono::high_resolution_clock::now();

		Sai2Model::graspMatrixAtGeometricCenter(G, R_g, geometric_center, contact_locations, constrained_rotations);

		// cout << "compute G" << endl;
		// cout << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << endl;
		duration = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();
		G_computation_time += duration;

		start = chrono::high_resolution_clock::now();
		G_inv = G.inverse();

		// cout << "compute G inverse" << endl;
		// cout << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << endl;
		duration = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();
		Ginv_computation_time += duration;


		VectorXd v_test_ginv = VectorXd::Zero(12);
		v_test_ginv << 1,2,3,4,5,6,7,8,9,0,1,2;
		start = chrono::high_resolution_clock::now();
		VectorXd res1 = G.colPivHouseholderQr().solve(v_test_ginv);
		VectorXd res2 = G.colPivHouseholderQr().solve(v_test_ginv);

		// cout << "solve 2 linear eq with G" << endl;
		// cout << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << endl;
		duration = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();
		Glin_solve_computation_time += duration;

		start = chrono::high_resolution_clock::now();
		MatrixXd G_inv_explicit = computeGInverseAtGeometricCenterExplicit(R_g, contact_locations[0], contact_locations[1], constrained_rotations[0], constrained_rotations[1]);
		
		// cout << "compute G inverse explicit" << endl;
		// cout << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << endl;
		duration = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();
		Ginv_explicit_computation_time += duration;

		// cout << "verify that g inverse explicit is correct :" << endl;
		// cout << "I - GinvG :\n" << (MatrixXd::Identity(12,12) - G_inv_explicit * G).norm() << endl; 
		// cout << "I - GGinv :\n" << (MatrixXd::Identity(12,12) - G * G_inv_explicit).norm() << endl; 

		double err1 = (MatrixXd::Identity(12,12) - G_inv_explicit * G).norm();
		double err2 = (MatrixXd::Identity(12,12) - G * G_inv_explicit).norm();

		if(err1 > max_error)
		{
			max_error = err1;
		}
		if(err2 > max_error)
		{
			max_error = err2;
		}
	}

	cout << "computation G :\n" << G_computation_time/n_iterations << endl;
	cout << "computation Ginv :\n" << Ginv_computation_time/n_iterations << endl;
	cout << "computation G lin :\n" << Glin_solve_computation_time/n_iterations << endl;
	cout << "computation G inv expl :\n" << Ginv_explicit_computation_time/n_iterations << endl;

	cout << "max error :\n" << max_error << endl;

	// cout << "geometric center :\n" << geometric_center.transpose() << endl;
	// cout << "G :\n" << G << endl;
	// cout << "G inverse :\n" << G_inv << endl;

	MatrixXd W = G.block(0,0,6,12);
	MatrixXd K = G.block(6,0,6,12);
	MatrixXd Wbar = G_inv.block(0,0,12,6);
	MatrixXd Kbar = G_inv.block(0,6,12,6);

	// cout << "W :\n" << W << endl;
	// cout << "K :\n" << K << endl;
	// cout << "Wbar :\n" << Wbar << endl;
	// cout << "Kbar :\n" << Kbar << endl;

	MatrixXd JC1, JC2;
	robots[0]->J_0WorldFrame(JC1, link_name, pos_in_link);
	robots[1]->J_0WorldFrame(JC2, link_name, pos_in_link);
	MatrixXd J_tot = MatrixXd::Zero(12,14);
	J_tot.block<3,7>(0,0) = JC1.block<3,7>(0,0);
	J_tot.block<3,7>(3,7) = JC2.block<3,7>(0,0);
	J_tot.block<3,7>(6,0) = JC1.block<3,7>(3,0);
	J_tot.block<3,7>(9,7) = JC2.block<3,7>(3,0);

	// cout << "JC1 :\n" << JC1 << endl;
	// cout << "JC2 :\n" << JC2 << endl;
	// cout << "J tot :\n" << J_tot << endl;

	// find geometric center position in lical link frames
	Affine3d T_world_hand1, T_world_hand2;
	robots[0]->transformInWorld(T_world_hand1, link_name);
	robots[1]->transformInWorld(T_world_hand2, link_name);
	Vector3d geometric_center_in_robot1_ee = T_world_hand1.inverse() * geometric_center;
	Vector3d geometric_center_in_robot2_ee = T_world_hand2.inverse() * geometric_center;

	// cout << "geom center in robot 1 ee :\n" << geometric_center_in_robot1_ee.transpose() << endl;
	// cout << "geom center in robot 2 ee :\n" << geometric_center_in_robot2_ee.transpose() << endl;

	MatrixXd JP1, JP2;
	robots[0]->J_0WorldFrame(JP1, link_name, geometric_center_in_robot1_ee);
	robots[1]->J_0WorldFrame(JP2, link_name, geometric_center_in_robot2_ee);

	// cout << "JP1 :\n" << JP1 << endl;
	// cout << "JP2 :\n" << JP2 << endl;

	Vector3d r_C1P = geometric_center - contact_locations[0];
	Vector3d r_C2P = geometric_center - contact_locations[1];
	Matrix3d r_C1P_cross = Sai2Model::CrossProductOperator(r_C1P);
	Matrix3d r_C2P_cross = Sai2Model::CrossProductOperator(r_C2P);

	// cout << "r C1P :\n" << r_C1P.transpose() << endl;
	// cout << "r C2P :\n" << r_C2P.transpose() << endl;

	MatrixXd E1 = MatrixXd::Identity(6,6);
	MatrixXd E2 = MatrixXd::Identity(6,6);
	E1.block<3,3>(0,3) = -r_C1P_cross;
	E2.block<3,3>(0,3) = -r_C2P_cross;

	// cout << "E1 :\n" << E1 << endl;
	// cout << "E2 :\n" << E2 << endl;

	// cout << "norm of E1*JC1 - JP1 (should be zero) :\n" << (E1*JC1 - JP1).norm() << endl;
	// cout << "norm of E2*JC2 - JP2 (should be zero) :\n" << (E2*JC2 - JP2).norm() << endl;

	MatrixXd E_tot = MatrixXd::Zero(6,12);
	E_tot.block<3,3>(0,0) = Matrix3d::Identity();
	E_tot.block<3,3>(0,3) = Matrix3d::Identity();
	E_tot.block<3,3>(3,6) = Matrix3d::Identity();
	E_tot.block<3,3>(3,9) = Matrix3d::Identity();
	E_tot.block<3,3>(0,6) = -r_C1P_cross;
	E_tot.block<3,3>(0,9) = -r_C2P_cross;

	MatrixXd Jr = Wbar.transpose() * J_tot;
	MatrixXd Jri = G_inv.transpose() * J_tot;	

	// cout << "Jr :\n" << Jr << endl;
	// cout << "E_tot*J_tot :\n" << E_tot*J_tot << endl;
	// cout << "norm of Jr - 0.5*E_tot*J_tot (should be zero ? NO) \n" << (Jr - 0.5 * E_tot * J_tot).norm() << endl;

	MatrixXd M_tot = MatrixXd::Zero(14,14);
	M_tot.block<7,7>(0,0) = robots[0]->_M;
	M_tot.block<7,7>(7,7) = robots[1]->_M;
	MatrixXd M_tot_inv = M_tot.inverse();

	MatrixXd Lambda_r_inv = Jr * M_tot_inv * Jr.transpose();
	MatrixXd Lambda_r = Lambda_r_inv.inverse();

	MatrixXd Lambda_ri_inv = Jri * M_tot_inv * Jri.transpose();
	MatrixXd Lambda_ri = Lambda_ri_inv.inverse();

	MatrixXd Lambda_sum_inv = E_tot*J_tot * M_tot_inv * J_tot.transpose()*E_tot.transpose();
	MatrixXd Lambda_sum = Lambda_sum_inv.inverse();

	MatrixXd Lambda1_inv = JP1 * robots[0]->_M_inv * JP1.transpose();
	MatrixXd Lambda2_inv = JP2 * robots[1]->_M_inv * JP2.transpose();
	MatrixXd Lambda1 = Lambda1_inv.inverse();
	MatrixXd Lambda2 = Lambda2_inv.inverse();

	// cout << "Lambda r :\n" << Lambda_r << endl;
	// cout << "Lambda sum :\n" << Lambda_sum << endl;
	// cout << "Lambda1 :\n" << Lambda1 << endl;
	// cout << "Lambda2 :\n" << Lambda2 << endl;
	// cout << "Lambda1 + Lambda2 :\n" << Lambda1 + Lambda2 << endl;
	// cout << "Lambda1 - Lambda2 :\n" << Lambda1 - Lambda2 << endl;
	// cout << "Lambda ri :\n" << Lambda_ri << endl;
	// cout << "diff of the top left part of Lambda ri and lambda_augmented :\n" << (Lambda_ri.block<6,6>(0,0) - Lambda1 - Lambda2).norm() << endl;

	MatrixXd Jbar_ri = M_tot_inv * Jri.transpose() * Lambda_ri;
	MatrixXd N_ri = MatrixXd::Identity(14,14) - Jbar_ri*Jri;

	// cout << "N ri :\n" << N_ri << endl;
	// cout << "norm of top right and bottom left parts of N ri :\n" << N_ri.block<7,7>(0,7).norm() << endl << N_ri.block<7,7>(7,0).norm() << endl;

	// two hand task
	// sf::safe_ptr<Sai2Primitives::TwoHandTwoRobotsTask> two_hand_task;
	// auto two_hand_task = sf::safe_ptr<Sai2Primitives::TwoHandTwoRobotsTask>(robots[0].get_obj_ptr(), robots[1].get_obj_ptr(), 
			// link_name, link_name,
			// pos_in_link, pos_in_link);
	// two_hand_task->_internal_force_control_flag = false;

	// two_hand_task->_use_interpolation_pos_flag = false;
	// two_hand_task->_use_interpolation_ori_flag = false;

	// // two_hand_task->_kp_pos = 0;
	// // two_hand_task->_kv_pos = 0;
	// // two_hand_task->_kp_ori = 0;
	// // two_hand_task->_kv_ori = 0;

	// // two_hand_task->_kp_internal_separation = 0;
	// // two_hand_task->_kv_internal_separation = 0;
	// // two_hand_task->_kp_internal_ori = 0;
	// // two_hand_task->_kv_internal_ori = 0;

	// // object properties
	// double object_mass = 1.0;
	// Matrix3d object_inertia = 0.1 * Matrix3d::Identity();

	// two_hand_task->updateTaskModel(MatrixXd::Identity(7,7),MatrixXd::Identity(7,7));

	return 0;
}

MatrixXd computeGInverseAtGeometricCenterExplicit(const Matrix3d Rg, 
		const Vector3d contact_location_1,
		const Vector3d contact_location_2)
{
	Vector3d x = Rg.col(0);
	Vector3d y = Rg.col(1);
	Vector3d z = Rg.col(2);

	double l = (contact_location_2 - contact_location_1).norm();
	if(l < 1e-5)
	{
		throw std::runtime_error("contact points should not overlap in computeGInverseAtGeometricCenterExplicit");
	}
	Vector3d x_expected = (contact_location_2 - contact_location_1)/l;
	if(1 - x_expected.dot(x) > 1e-5)
	{
		throw std::runtime_error("Rg not consistent with the contact locations given in computeGInverseAtGeometricCenterExplicit");
	}

	Matrix3d x_cross = Sai2Model::CrossProductOperator(x);
	Matrix3d x_cross_square = x_cross * x_cross;

	MatrixXd Wbar = MatrixXd::Zero(12,6);
	MatrixXd Kbar = MatrixXd::Zero(12,6);

	Wbar.block<3,3>(0,0) = Matrix3d::Identity() / 2.0;	
	Wbar.block<3,3>(3,0) = Matrix3d::Identity() / 2.0;	
	Wbar.block<3,3>(0,3) = x_cross / l;
	Wbar.block<3,3>(3,3) = -x_cross / l;
	Wbar.block<3,3>(6,3) = (Matrix3d::Identity() + x_cross_square) / 2.0;
	Wbar.block<3,3>(9,3) = (Matrix3d::Identity() + x_cross_square) / 2.0;

	Kbar.block<3,1>(0,0) = -x;
	Kbar.block<3,1>(3,0) = x;
	Kbar.block<3,1>(6,1) = -x;
	Kbar.block<3,1>(9,1) = x;

	Kbar.block<3,1>(0,2) = -z/l;
	Kbar.block<3,1>(3,2) = z/l;
	Kbar.block<3,1>(6,2) = y;

	Kbar.block<3,1>(0,3) = y/l;
	Kbar.block<3,1>(3,3) = -y/l;
	Kbar.block<3,1>(6,3) = z;

	Kbar.block<3,1>(0,4) = -z/l;
	Kbar.block<3,1>(3,4) = z/l;
	Kbar.block<3,1>(9,4) = y;

	Kbar.block<3,1>(0,5) = y/l;
	Kbar.block<3,1>(3,5) = -y/l;
	Kbar.block<3,1>(9,5) = z;

	MatrixXd Ginv = MatrixXd::Zero(12,12);

	Ginv.block<12,6>(0,0) = Wbar;
	Ginv.block<12,6>(0,6) = Kbar;

	return Ginv;
}
