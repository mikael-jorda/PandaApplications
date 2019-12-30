// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "haptic_tasks/OpenLoopTeleop.h"

#include <iostream>
#include <string>

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

const int n_robots = 2;

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

const bool inertia_regularization = true;

#define GOTO_INITIAL_CONFIG               0
#define HAPTIC_CONTROL                    1

int translation_counter = 0;

int remote_enabled = 1; //////////////////////////////////////////////////////// Read from redis ??
int restart_cycle = 0;

int state_right = GOTO_INITIAL_CONFIG;
int state_left = GOTO_INITIAL_CONFIG;

unsigned long long controller_counter = 0;

RedisClient redis_client;

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


	// position of robots in world
	vector<Affine3d> robot_pose_in_world;
	Affine3d pose = Affine3d::Identity();
	pose.translation() = Vector3d(0, -0.5, 0.0);
	pose.linear() = AngleAxisd(0.3010693, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	pose.translation() = Vector3d(-0.06, 0.57, 0.0);
	pose.linear() = AngleAxisd(-1.0864675, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	auto redis_client = RedisClient();
	redis_client.connect();

	redis_client.set(REMOTE_ENABLED_KEY, to_string(remote_enabled));
	redis_client.set(RESTART_CYCLE_KEY, to_string(restart_cycle));

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	vector<Sai2Model::Sai2Model*> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(new Sai2Model::Sai2Model(robot_files[i], false));
		robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
		robots[i]->updateModel();
	}

	// prepare robot task controllers
	vector<int> dof;
	vector<VectorXd> command_torques;
	vector<VectorXd> coriolis;
	vector<MatrixXd> N_prec;

	vector<Sai2Primitives::JointTask*> joint_tasks;
	vector<VectorXd> joint_task_torques;
	vector<Sai2Primitives::PosOriTask*> posori_tasks;
	vector<VectorXd> posori_task_torques;

	vector<VectorXd> q_initial;
	VectorXd q_init_1 = VectorXd::Zero(7);
	VectorXd q_init_2 = VectorXd::Zero(7);
	q_init_1 << 0.585135,-1.04046,0.286799,-2.59264,1.95629,1.95654,-1.45693;
	q_init_2 << 2.22268,1.14805,-1.90333,-2.21937,-0.645465,1.70329,-0.074406;

	q_initial.push_back(q_init_1);
	q_initial.push_back(q_init_2);

	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		command_torques.push_back(VectorXd::Zero(dof[i]));
		coriolis.push_back(VectorXd::Zero(dof[i]));
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));

		// joint tasks
		joint_tasks.push_back(new Sai2Primitives::JointTask(robots[i]));
		joint_task_torques.push_back(VectorXd::Zero(dof[i]));

		joint_tasks[i]->_kp = 100.0;
		joint_tasks[i]->_kv = 14.0;
		// joint_tasks[i]->_ki = 50.0;
		joint_tasks[i]->_use_velocity_saturation_flag = true;
		joint_tasks[i]->_desired_position = q_initial[i];
		// joint_tasks[i]->_use_interpolation_flag = false;
		// joint_tasks[i]->_use_velocity_saturation_flag = true;
		// joint_tasks[i]->_saturation_velocity = M_PI/6.0 * VectorXd::Ones(7);
		joint_tasks[i]->_otg->setMaxVelocity(M_PI/6);
		// cout << "desired pos : " << q_initial[i].transpose() << endl;
		// cout << "current pos : " << joint_tasks[i]->_current_position.transpose() << endl;

		// end effector tasks
		string link_name = "link7";
		Eigen::Vector3d pos_in_link = Vector3d(0.0,0.0,0.2);
		posori_tasks.push_back(new Sai2Primitives::PosOriTask(robots[i], link_name, pos_in_link));
		posori_task_torques.push_back(VectorXd::Zero(dof[i]));
		posori_tasks[i]->_use_interpolation_flag = false;
		posori_tasks[i]->_use_velocity_saturation_flag = true;
		posori_tasks[i]->_linear_saturation_velocity = 0.5;
		posori_tasks[i]->_angular_saturation_velocity = M_PI/1.5;
		// posori_tasks[i]->_otg->setMaxLinearVelocity(0.5);
		// posori_tasks[i]->_otg->setMaxAngularVelocity(2.0*M_PI/3.0);

		posori_tasks[i]->_kp_pos = 100.0;
		posori_tasks[i]->_kv_pos = 15.0;
		posori_tasks[i]->_kp_ori = 200.0;
		posori_tasks[i]->_kv_ori = 15.0;		
	}
	// define home positions of the panda robots
	Vector3d workspace_center_left = posori_tasks[0]->_current_position;
	Vector3d workspace_center_right = posori_tasks[1]->_current_position;

	// define home position of the haptic device
	Vector3d haptic_center_left = Vector3d::Zero();
	Vector3d haptic_center_right = Vector3d::Zero();


	// connect haptic device and prepare teleoperation controller
	auto handler = new cHapticDeviceHandler();

	//Robot Brush : Robot[0] - LEFT && Robot Palette : Robot[1] - RIGHT
	Matrix3d transformDev_Rob_left = robot_pose_in_world[0].linear(); //////////////////////////////////////////
	auto haptic_controller_left = new Sai2Primitives::OpenLoopTeleop(handler, 0, workspace_center_left, posori_tasks[0]->_current_orientation, transformDev_Rob_left);
	haptic_controller_left->initializeSigmaDevice();
	haptic_controller_left->GravityCompTask();
	//Task scaling factors
	double Ks_brush=1.0;
	double KsR_brush=1.0;
	haptic_controller_left->setScalingFactors(Ks_brush, KsR_brush);

	double _pos_gripper=0.0;


	Matrix3d transformDev_Rob_right = robot_pose_in_world[1].linear(); ///////////////////////////////////////////////
	auto haptic_controller_right = new Sai2Primitives::OpenLoopTeleop(handler, 1, workspace_center_right, posori_tasks[1]->_current_orientation, transformDev_Rob_right);
	haptic_controller_right->initializeSigmaDevice();
	haptic_controller_right->GravityCompTask();
	//Task scaling factors
	double Ks_right=1.0;
	double KsR_right=1.0;
	haptic_controller_right->setScalingFactors(Ks_right, KsR_right);

	VectorXd f_task_sensed = VectorXd::Zero(6);

	Vector3d pos_proxy_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_proxy_model = Matrix3d::Identity();
	Vector3d vel_rot_proxy_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_proxy_model = Vector3d::Zero();

	// Force feedback stiffness proxy parameters
	double k_pos = 3000.0;
	double d_pos = 1.0;
	double k_ori = 20.0;
	double d_ori = 0.1;
	Matrix3d Red_factor_rot = Matrix3d::Identity();
	Matrix3d Red_factor_trans = Matrix3d::Identity();
	Red_factor_rot << 1/10.0, 0.0, 0.0,
						  0.0, 1/10.0, 0.0,
						  0.0, 0.0, 1/10.0;

	Red_factor_trans << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;
	haptic_controller_right->setForceFeedbackCtrlGains (k_pos, d_pos, k_ori, d_ori, Red_factor_rot, Red_factor_trans);
	
	haptic_controller_right->_filter_on = false;
	double fc_force=0.02;
	double fc_moment=0.02;
	haptic_controller_right->setFilterCutOffFreq(fc_force, fc_moment);


	bool gripper_state_right = false;
	haptic_controller_right->EnableGripperUserSwitch();

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

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

					// update model
			if(flag_simulation)
			{
				robots[i]->updateModel();
				robots[i]->coriolisForce(coriolis[i]);
			}
			else
			{
				robots[i]->updateKinematics();
				robots[i]->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEYS[i]);
				if(inertia_regularization)
				{
					robots[i]->_M(4,4) += 0.07;
					robots[i]->_M(5,5) += 0.07;
					robots[i]->_M(6,6) += 0.07;
				}
				robots[i]->_M_inv = robots[i]->_M.inverse();

				coriolis[i] = redis_client.getEigenMatrixJSON(CORIOLIS_KEYS[i]);
			}
		}

		// Update position and orientation of the robot from the model for proxy
		robot[1]->position(pos_proxy_model, proxy_link, poxy_pos_in_link);
		robot[1]->linearVelocity(vel_trans_proxy_model, proxy_link, poxy_pos_in_link);
		robot[1]->rotation(rot_proxy_model, proxy_link);
		robot[1]->angularVelocity(vel_rot_proxy_model, proxy_link);

		// read force sensor data and remove bias and effecto from hand gravity
		f_task_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEYS[0]); /////////////////////////////////////////////////

		// read renabling/disabling teleoperation brush
		remote_enabled = stoi(redis_client.get(REMOTE_ENABLED_KEY));
		// read restar cycle key
		restart_cycle = stoi(redis_client.get(RESTART_CYCLE_KEY));

///////////////////////////////////////////////////////////////////////////////////////////
		//// State Machine Robot 0 - Haptic LEFT -brush////
		if(state_left == GOTO_INITIAL_CONFIG)
		{
			// update robot home position task model
			N_prec[0].setIdentity();
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			// compute robot torques
			joint_tasks[0]->computeTorques(joint_task_torques[0]);
			command_torques[0] = joint_task_torques[0] + coriolis[0];
			
			// compute homing haptic device
			haptic_controller_left->HomingTask();

			// cout << "left robot error : " << (joint_tasks[0]->_desired_position - joint_tasks[0]->_current_position).norm() << endl;

			if(remote_enabled==1 && (joint_tasks[0]->_desired_position - joint_tasks[0]->_current_position).norm() < 0.2 && haptic_controller_left->device_homed)
			{
				joint_tasks[0]->reInitializeTask();
				posori_tasks[0]->reInitializeTask();
				workspace_center_left = posori_tasks[0]->_current_position;
				haptic_center_left = haptic_controller_left->_current_position_device ;

				haptic_controller_left->setRobotCenter(workspace_center_left, posori_tasks[0]->_current_orientation);
				haptic_controller_left->setDeviceCenter(haptic_center_left, haptic_controller_left->_current_rotation_device);
				
				state_left = HAPTIC_CONTROL;
			}
		}

		else if(state_left == HAPTIC_CONTROL)
		{

			// update tasks model
			N_prec[0].setIdentity();
			posori_tasks[0]->updateTaskModel(N_prec[0]);
			N_prec[0] = posori_tasks[0]->_N;
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			
			haptic_controller_left->_send_haptic_feedback = false;

			haptic_controller_left->computeHapticCommands6d(posori_tasks[0]->_desired_position,
																   posori_tasks[0]->_desired_orientation);

			haptic_controller_left->hapticDevice->getGripperAngRad(_pos_gripper);

			// compute robot set torques
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			command_torques[0] = joint_task_torques[0] + coriolis[0] + posori_task_torques[0];

		}
		else
		{
			command_torques[0].setZero(dof[0]);
			haptic_controller_left->GravityCompTask();
		}


///////////////////////////////////////////////////////////////////////////////////////////
		//// State Machine Robot 1 - Haptic Palette ////
		if(state_right == GOTO_INITIAL_CONFIG)
		{
			// update robot home position task model
			N_prec[1].setIdentity();
			joint_tasks[1]->updateTaskModel(N_prec[1]);
			// compute robot torques
			joint_tasks[1]->computeTorques(joint_task_torques[1]);
			command_torques[1] = joint_task_torques[1] + coriolis[1];
			
			// compute homing haptic device
			haptic_controller_right->HomingTask();

			// read gripper state
			gripper_state_right = haptic_controller_right->ReadGripperUserSwitch();

			if(remote_enabled==1 && (joint_tasks[1]->_desired_position - joint_tasks[1]->_current_position).norm() < 0.2 && haptic_controller_right->device_homed)
			{
				joint_tasks[1]->reInitializeTask();
				posori_tasks[1]->reInitializeTask();
				workspace_center_right = posori_tasks[1]->_current_position;
				haptic_center_right = haptic_controller_right->_current_position_device;

				haptic_controller_right->setRobotCenter(workspace_center_right, posori_tasks[1]->_current_orientation);
				haptic_controller_right->setDeviceCenter(haptic_center_right, haptic_controller_right->_current_rotation_device);
				
				state_right = HAPTIC_CONTROL;
			}
		}

		else if(state_right == HAPTIC_CONTROL)
		{

			// update tasks model
			N_prec[1].setIdentity();
			posori_tasks[1]->updateTaskModel(N_prec[1]);
			N_prec[1] = posori_tasks[1]->_N;
			joint_tasks[1]->updateTaskModel(N_prec[1]);

			//Compute haptic commands
			haptic_controller_right->_haptic_feedback_from_proxy = true; // If set to true, the force feedback is computed from a stiffness/damping proxy.
	    	haptic_controller_right->_filter_on = false; // filtering parameters are tuned in the initialization
			
			//haptic_controller_right->updateSensedForce(f_task_sensed);
			haptic_controller_right->updateSensedRobotPositionVelocity(pos_proxy_model, vel_trans_proxy_model, rot_proxy_model, vel_rot_proxy_model);


			haptic_controller_right->_send_haptic_feedback = true;


			haptic_controller_right->computeHapticCommands6d(posori_tasks[1]->_desired_position,
																   posori_tasks[1]->_desired_orientation);


			// compute robot set torques
			posori_tasks[1]->computeTorques(posori_task_torques[1]);
			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			command_torques[1] = joint_task_torques[1] + coriolis[1] + posori_task_torques[1];

			// read gripper state
			gripper_state_right = haptic_controller_right->ReadGripperUserSwitch();

		}
		else
		{
			command_torques[1].setZero(dof[1]);
			haptic_controller_left->GravityCompTask();

		}
///////////////////////////////////////////////////////////////////////////////////////////

		

		// send to redis
		for(int i=0 ; i<n_robots ; i++)
		{
			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
		}

		prev_time = current_time;
		controller_counter++;
	}

	for(int i=0 ; i<n_robots ; i++)
	{
		command_torques[i].setZero();
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
	}
	delete haptic_controller_right;
	delete haptic_controller_left;

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

// VectorXd readBiasXML(const string path_to_bias_file)
// {
// 	VectorXd sensor_bias = VectorXd::Zero(6);
// 	tinyxml2::XMLDocument doc;
// 	doc.LoadFile(path_to_bias_file.c_str());
// 	if (!doc.Error())
// 	{
// 		cout << "Loading bias file file ["+path_to_bias_file+"]." << endl;
// 		try 
// 		{

// 			std::stringstream bias( doc.FirstChildElement("force_bias_global")->
// 				Attribute("value"));
// 			bias >> sensor_bias(0);
// 			bias >> sensor_bias(1);
// 			bias >> sensor_bias(2);
// 			bias >> sensor_bias(3);
// 			bias >> sensor_bias(4);
// 			bias >> sensor_bias(5);
// 			std::stringstream ss; ss << sensor_bias.transpose();
// 			cout << "Sensor bias : "+ss.str() << endl;
// 		}
// 		catch( const std::exception& e ) // reference to the base of a polymorphic object
// 		{ 
// 			std::cout << e.what(); // information from length_error printed
// 			cout << "WARNING : Failed to parse bias file." << endl;
// 		}
// 	} 
// 	else 
// 	{
// 		cout << "WARNING : Could no load bias file ["+path_to_bias_file+"]" << endl;
// 		doc.PrintError();
// 	}
// 	return sensor_bias;
// }