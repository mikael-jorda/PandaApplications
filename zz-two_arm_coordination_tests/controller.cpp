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

const int n_robots = robot_names.size();

// redis keys:
// - read:
const vector<string> JOINT_ANGLES_KEYS  = {
	"sai2::PandaApplications::panda1::sensors::q",
	"sai2::PandaApplications::panda2::sensors::q",
};
const vector<string> JOINT_VELOCITIES_KEYS = {
	"sai2::PandaApplications::panda1::sensors::dq",
	"sai2::PandaApplications::panda2::sensors::dq",
};

const vector<string> SENSED_FORCES_KEYS = {
	"sai2::PandaApplications::panda1::sensors::wrist_force_moment",
	"sai2::PandaApplications::panda2::sensors::wrist_force_moment",
};

const string OBJECT_POSITION_KEY = "sai2::PandaApplications::object_position";

// - write
const vector<string> TORQUES_COMMANDED_KEYS = {
	"sai2::PandaApplications::panda1::actuators::fgc",
	"sai2::PandaApplications::panda2::actuators::fgc",
};

// function to update model at a slower rate
void updateModelThread(vector<sf::safe_ptr<Sai2Model::Sai2Model>> robots, 
		vector<sf::safe_ptr<Sai2Primitives::JointTask>> joint_tasks, 
		vector<sf::safe_ptr<Sai2Primitives::PosOriTask>> posori_tasks,
		sf::safe_ptr<Sai2Primitives::TwoHandTwoRobotsTask> two_hand_task);

// state machine
#define INDEPENDANT_ARMS                       0
#define COORDINATED_ARMS                       1
#define COORDINATED_ARMS_INTERNAL_FORCE        2

int state = INDEPENDANT_ARMS;
unsigned long long controller_counter = 0;

RedisClient redis_client;

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


	// create redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// position of robots in world
	vector<Affine3d> robot_pose_in_world;
	Affine3d pose = Affine3d::Identity();
	pose.translation() = Vector3d(0, -0.5, 0.0);
	pose.linear() = AngleAxisd(0.3010693, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	pose.translation() = Vector3d(-0.06, 0.57, 0.0);
	pose.linear() = AngleAxisd(-1.0864675, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	vector<sf::safe_ptr<Sai2Model::Sai2Model>> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(sf::safe_ptr<Sai2Model::Sai2Model>(robot_files[i], false, robot_pose_in_world[i]));
		robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
		robots[i]->updateModel();
	}

	// prepare task controllers
	vector<int> dof;
	vector<VectorXd> command_torques;
	vector<VectorXd> coriolis;
	vector<MatrixXd> N_prec;

	vector<sf::safe_ptr<Sai2Primitives::JointTask>> joint_tasks;
	vector<VectorXd> joint_task_torques;
	vector<sf::safe_ptr<Sai2Primitives::PosOriTask>> posori_tasks;
	vector<VectorXd> posori_task_torques;
	vector<VectorXd> two_hand_task_torques;


	VectorXd sensed_force_moment_1 = VectorXd::Zero(6);
	VectorXd sensed_force_moment_2 = VectorXd::Zero(6);

	const string link_name = "link7";
	// const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.125);
	const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.115);

	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		command_torques.push_back(VectorXd::Zero(dof[i]));
		coriolis.push_back(VectorXd::Zero(dof[i]));
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));

		// joint tasks
		joint_tasks.push_back(sf::safe_ptr<Sai2Primitives::JointTask>(robots[i].get_obj_ptr()));
		joint_task_torques.push_back(VectorXd::Zero(dof[i]));

		joint_tasks[i]->_kp = 50.0;
		joint_tasks[i]->_kv = 14.0;

		// end effector tasks
		posori_tasks.push_back(sf::safe_ptr<Sai2Primitives::PosOriTask>(robots[i].get_obj_ptr(), link_name, pos_in_link));
		posori_task_torques.push_back(VectorXd::Zero(dof[i]));

		posori_tasks[i]->_kp_pos = 200.0;
		posori_tasks[i]->_kv_pos = 25.0;
		posori_tasks[i]->_kp_ori = 200.0;
		posori_tasks[i]->_kv_ori = 20.0;		

		posori_tasks[i]->_use_interpolation_flag = true;

		posori_tasks[i]->_otg->setMaxLinearVelocity(0.4);
		posori_tasks[i]->_otg->setMaxLinearAcceleration(1.0);
		posori_tasks[i]->_otg->setMaxLinearJerk(3.0);

		posori_tasks[i]->_otg->setMaxAngularVelocity(M_PI);
		posori_tasks[i]->_otg->setMaxAngularAcceleration(M_PI);
		posori_tasks[i]->_otg->setMaxAngularJerk(3*M_PI);

		// two hand task torques
		two_hand_task_torques.push_back(VectorXd::Zero(dof[i]));

	}

	// two hand task
	// sf::safe_ptr<Sai2Primitives::TwoHandTwoRobotsTask> two_hand_task;
	auto two_hand_task = sf::safe_ptr<Sai2Primitives::TwoHandTwoRobotsTask>(robots[0].get_obj_ptr(), robots[1].get_obj_ptr(), 
			posori_tasks[0]->_link_name, posori_tasks[1]->_link_name,
			posori_tasks[0]->_control_frame, posori_tasks[1]->_control_frame);
	two_hand_task->_internal_force_control_flag = false;

	two_hand_task->_use_interpolation_pos_flag = false;
	two_hand_task->_use_interpolation_ori_flag = false;

	// two_hand_task->_kp_pos = 0;
	// two_hand_task->_kv_pos = 0;
	// two_hand_task->_kp_ori = 0;
	// two_hand_task->_kv_ori = 0;

	// two_hand_task->_kp_internal_separation = 0;
	// two_hand_task->_kv_internal_separation = 0;
	// two_hand_task->_kp_internal_ori = 0;
	// two_hand_task->_kv_internal_ori = 0;

	// object properties
	double object_mass = 1.0;
	Matrix3d object_inertia = 0.1 * Matrix3d::Identity();

	// set the sensor frames
	string sensor_link_name = "link7";
	Affine3d T_link_sensor = Affine3d::Identity();
	T_link_sensor.translation() = Vector3d(0, 0, 0.113);
	two_hand_task->setForceSensorFrames(sensor_link_name, T_link_sensor, sensor_link_name, T_link_sensor);

	// set goal positions for the first state in world frame
	Vector3d robot1_desired_position_in_world = Vector3d(0.2, -0.2, 0.55);
	Vector3d robot2_desired_position_in_world = Vector3d(0.2,  0.2, 0.55);

	Matrix3d robot1_desired_orientation_in_world;
	Matrix3d robot2_desired_orientation_in_world;
	robot1_desired_orientation_in_world << 1, 0, 0, 0, 0, 1, 0, -1, 0;
	robot2_desired_orientation_in_world << 1, 0, 0, 0, 0, -1, 0, 1, 0;

	// robot1_desired_orientation_in_world << 0, 0, 1, 1, 0, 0, 0, 1, 0;
	// robot2_desired_orientation_in_world << 0, 0, 1, 1, 0, 0, 0, 1, 0;

	// robot1_desired_orientation_in_world *= AngleAxisd(M_PI/4, Vector3d::UnitY()).toRotationMatrix();

	// set desired position and orientation for posori tasks : needs to be in robot frame
	posori_tasks[0]->_desired_position = robot_pose_in_world[0].linear().transpose()*(robot1_desired_position_in_world - robot_pose_in_world[0].translation());
	posori_tasks[0]->_desired_orientation = robot_pose_in_world[0].linear().transpose()*robot1_desired_orientation_in_world;
	posori_tasks[1]->_desired_position = robot_pose_in_world[1].linear().transpose()*(robot2_desired_position_in_world - robot_pose_in_world[1].translation());
	posori_tasks[1]->_desired_orientation = robot_pose_in_world[1].linear().transpose()*robot2_desired_orientation_in_world;

	// start update_model thread
	thread model_update_thread(updateModelThread, robots, joint_tasks, posori_tasks, two_hand_task);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	double initial_time = 0;

	runloop = true;
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;
		dt = current_time - prev_time;

		// read robot state from redis and update robot model
		for(int i=0 ; i<n_robots ; i++)
		{
			robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
			robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);

			robots[i]->updateKinematics();
			robots[i]->coriolisForce(coriolis[i]);
		}
		sensed_force_moment_1 = redis_client.getEigenMatrixJSON(SENSED_FORCES_KEYS[0]);
		sensed_force_moment_2 = redis_client.getEigenMatrixJSON(SENSED_FORCES_KEYS[1]);

		// go to box pre grasp position
		if(state == INDEPENDANT_ARMS)
		{

			// compute torques
			for(int i=0 ; i<n_robots ; i++)
			{
				posori_tasks[i]->computeTorques(posori_task_torques[i]);
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i];
			}

			// when the goal is reached, switch to two hand coordinated task
			if(posori_tasks[0]->goalPositionReached(1e-2) &&
				posori_tasks[1]->goalPositionReached(1e-2) &&
				posori_tasks[0]->goalOrientationReached(1e-2) &&
				posori_tasks[1]->goalOrientationReached(1e-2))
			{
				// set control frame for the object
				Affine3d T_world_controlpoint = Affine3d::Identity();
				T_world_controlpoint.translation() = Vector3d(0.2, 0.0, 0.0);
				two_hand_task->setControlFrameLocationInitial(T_world_controlpoint);

				// re intialize tasks
				two_hand_task->reInitializeTask();
				joint_tasks[0]->reInitializeTask();
				joint_tasks[1]->reInitializeTask();

				// set desired internal tension for the object not to slip
				// two_hand_task->_desired_internal_tension = -1.0;
				// two_hand_task->_desired_internal_separation -= 0.15;
				// two_hand_task->_desired_internal_angles(0) = 1.5;

				// set the object properties for the two hand task
				// Affine3d T_world_com = Affine3d::Identity();
				// T_world_com.linear() = two_hand_task->_current_object_orientation;
				// T_world_com.translation() = two_hand_task->_current_object_position;
				// two_hand_task->setObjectMassPropertiesAndInitialInertialFrameLocation(object_mass, T_world_com, object_inertia);

				// change the desired position and orientation of the object
				// two_hand_task->_desired_object_position(2) += 0.20;
				// two_hand_task->_desired_object_orientation *= AngleAxisd(M_PI* 30.0/180.0, Vector3d::UnitZ()).toRotationMatrix();
				// two_hand_task->_desired_object_orientation = AngleAxisd(M_PI* 30.0/180.0, Vector3d::UnitZ()).toRotationMatrix() * two_hand_task->_desired_object_orientation;
				// two_hand_task->_desired_object_orientation = AngleAxisd(-M_PI* 10.0/180.0, Vector3d::UnitY()).toRotationMatrix() * two_hand_task->_desired_object_orientation;
				// two_hand_task->_desired_object_orientation = AngleAxisd(M_PI* 5.0/180.0, Vector3d::UnitX()).toRotationMatrix() * two_hand_task->_desired_object_orientation;

				// change the state
				state = COORDINATED_ARMS;

				// record initial time
				initial_time = current_time;
			}

		}

		// move the internal quantities
		else if(state == COORDINATED_ARMS)
		{
			auto start = chrono::high_resolution_clock::now();

			// update the sensed forces
			two_hand_task->updateSensedForcesAndMoments(sensed_force_moment_1.head(3), sensed_force_moment_1.tail(3),
						sensed_force_moment_2.head(3), sensed_force_moment_2.tail(3));
			
			// cout << "update sensed forces" << endl;
		    // cout << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << endl;

			// two_hand_task->_desired_internal_separation -= 0.00015;
			// two_hand_task->_desired_internal_angles(4) += 0.001;
			// two_hand_task->_desired_object_orientation *= AngleAxisd(0.0005, Vector3d::UnitY()).toRotationMatrix();

		    start = chrono::high_resolution_clock::now();

			// compute the torques
			two_hand_task->computeTorques(two_hand_task_torques[0], two_hand_task_torques[1]);
			
			// cout << "compute two hand torques" << endl;
		    // cout << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << endl;


			for(int i=0 ; i<n_robots ; i++)
			{
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = two_hand_task_torques[i] + joint_task_torques[i];
			}

		    start = chrono::high_resolution_clock::now();

			// cout << "compute joint torques" << endl;
		    // cout << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << endl;

		    cout << endl;

			JacobiSVD<MatrixXd> svd(two_hand_task->_grasp_matrix);
			double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
			// cout << "condition number grasp matrix : " << cond << endl;



			// // wait for a certain time
			// if(current_time - initial_time > 2.0)
			// {
			// 	// set the object properties for the two hand task
			// 	Affine3d T_world_com = Affine3d::Identity();
			// 	T_world_com.linear() = two_hand_task->_current_object_orientation;
			// 	T_world_com.translation() = two_hand_task->_current_object_position;
			// 	two_hand_task->setObjectMassPropertiesAndInitialInertialFrameLocation(object_mass, T_world_com, object_inertia);

			// 	// set the internal controller to force
			// 	two_hand_task->_internal_force_control_flag = true;
			// 	two_hand_task->_desired_internal_tension = -25.0;

			// 	// change the desired position and orientation of the object
			// 	two_hand_task->_desired_object_position(2) += 0.20;
			// 	two_hand_task->_desired_object_orientation = AngleAxisd(M_PI* 30.0/180.0, Vector3d::UnitZ()).toRotationMatrix() * two_hand_task->_desired_object_orientation;
			// 	two_hand_task->_desired_object_orientation = AngleAxisd(-M_PI* 10.0/180.0, Vector3d::UnitY()).toRotationMatrix() * two_hand_task->_desired_object_orientation;
			// 	two_hand_task->_desired_object_orientation = AngleAxisd(M_PI* 5.0/180.0, Vector3d::UnitX()).toRotationMatrix() * two_hand_task->_desired_object_orientation;

			// 	// change the state
			// 	state = COORDINATED_ARMS_INTERNAL_FORCE;

			// }
		}

		// // move the box to successive locations
		// else if(state == COORDINATED_ARMS_INTERNAL_FORCE)
		// {
		// 	// update the sensed forces
		// 	two_hand_task->updateSensedForcesAndMoments(sensed_force_moment_1.head(3), sensed_force_moment_1.tail(3),
		// 				sensed_force_moment_2.head(3), sensed_force_moment_2.tail(3));

		// 	// compute the torques
		// 	two_hand_task->computeTorques(two_hand_task_torques[0], two_hand_task_torques[1]);
			
		// 	for(int i=0 ; i<n_robots ; i++)
		// 	{
		// 		joint_tasks[i]->computeTorques(joint_task_torques[i]);

		// 		command_torques[i] = two_hand_task_torques[i] + joint_task_torques[i];
		// 	}

		// }

		else
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				command_torques[i].setZero(dof[i]);
			}
		}

		// send to redis
		// cout << controller_counter << endl;
		// cout << robots[0]->_q.transpose() << endl;
		// cout << two_hand_task->_current_internal_separation << endl;
		cout << endl;

		// cout << two_hand_task->_grasp_matrix << endl << endl;
		// cout << two_hand_task->_grasp_matrix.inverse() << endl << endl;


		if(command_torques[0].norm() > 500)
		{
			cout << "bf" << endl;
		}
		for(int i=0 ; i<n_robots ; i++)
		{
			// if(isnan(command_torques[i](0)))
			// {
				// cout << "nan" << endl;
				// command_torques[i].setZero();
			// }
			redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEYS[i], command_torques[i]);
		}

		prev_time = current_time;
		controller_counter++;
	}

	for(int i=0 ; i<n_robots ; i++)
	{
		command_torques[i].setZero();
		redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEYS[i], command_torques[i]);
	}

	model_update_thread.join();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

void updateModelThread(vector<sf::safe_ptr<Sai2Model::Sai2Model>> robots, 
		vector<sf::safe_ptr<Sai2Primitives::JointTask>> joint_tasks, 
		vector<sf::safe_ptr<Sai2Primitives::PosOriTask>> posori_tasks,
		sf::safe_ptr<Sai2Primitives::TwoHandTwoRobotsTask> two_hand_task)
{
	// prepare task controllers
	vector<int> dof;
	vector<MatrixXd> N_prec;

	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));
	}

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(50); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while(runloop)
	{
		timer.waitForNextLoop();
		for(int i=0 ; i<n_robots ; i++)
		{
			robots[i]->updateModel();
		}

		if(state == INDEPENDANT_ARMS)
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				posori_tasks[i]->updateTaskModel(N_prec[i]);

				N_prec[i] = posori_tasks[i]->_N;
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}
		}

		else if(state == COORDINATED_ARMS || state == COORDINATED_ARMS_INTERNAL_FORCE)
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
			}

			two_hand_task->updateTaskModel(N_prec[0], N_prec[1]);
			N_prec[0] = two_hand_task->_N_1;
			N_prec[1] = two_hand_task->_N_2;

			for(int i=0 ; i<n_robots ; i++)
			{
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}
		}
	}
}
