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
#define MAINTAIN_POSITION				  2

int translation_counter = 0;

int remote_enabled = 1; //////////////////////////////////////////////////////// Read from redis ??
int restart_cycle = 0;

int state_palette = GOTO_INITIAL_CONFIG;
int state_brush = GOTO_INITIAL_CONFIG;

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
	// q_init_1 << 0.44, -1.024, 0.274, -2.54, 1.945, 1.92, -1.526; 
	// q_init_2 << 2.24, 1.17, -2.047, -2.325, -0.584, 1.656, -0.03; 
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
	Vector3d workspace_center_brush = posori_tasks[0]->_current_position;
	Vector3d workspace_center_palette = posori_tasks[1]->_current_position;

	// define home position of the haptic device
	Vector3d haptic_center_brush = Vector3d::Zero();
	Vector3d haptic_center_palette = Vector3d::Zero();

	// connect haptic device and prepare teleoperation controller
	auto handler = new cHapticDeviceHandler();

	//Robot Brush : Robot[0] && Robot Palette : Robot[1]
	Matrix3d transformDev_Rob_brush = robot_pose_in_world[0].linear(); //////////////////////////////////////////
	auto haptic_controller_brush = new Sai2Primitives::OpenLoopTeleop(handler, 0, workspace_center_brush, posori_tasks[0]->_current_orientation, transformDev_Rob_brush);
	while(!haptic_controller_brush->device_started)
	{
		cout << "starting left device" << endl;
	}
	haptic_controller_brush->GravityCompTask();
	bool filter_on=true;
	haptic_controller_brush->setFilterCutOffFreq(0.04, 0.04);
	//Task scaling factors
	double Ks_brush=2.0;
	double KsR_brush=1.0;
	haptic_controller_brush->setScalingFactors(Ks_brush, KsR_brush);
	// Force feedback stiffness proxy parameters
	double k_pos_brush = 200.0;
	double k_ori_brush = 15.0;
	Matrix3d Red_factor_rot_brush = Matrix3d::Identity();
	Matrix3d Red_factor_trans_brush = Matrix3d::Identity();
	Red_factor_rot_brush << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	Red_factor_trans_brush << 1.0, 0.0, 0.0,
						  0.0, 1.0, 0.0,
						  0.0, 0.0, 1.0;
	haptic_controller_brush->setForceFeedbackCtrlGains(k_pos_brush, k_ori_brush, Red_factor_rot_brush, Red_factor_trans_brush);
	VectorXd f_task_sensed = VectorXd::Zero(6);
	// const VectorXd force_bias_global = readBiasXML("../../09-haptic_painting/Clyde_fsensor_bias.xml");
	VectorXd force_bias_global = VectorXd::Zero(6);
	force_bias_global << -2.32819,  -3.88484,   -129.07,  -0.38751,   1.57918, -0.021328;
	VectorXd force_bias_adjustment = VectorXd::Zero(6);
	const double hand_mass = 1.5;
	const Vector3d hand_com = Vector3d(0.0, 0.0, 0.1);
	// write task force in redis
	redis_client.setEigenMatrixJSON(FORCE_SENSED_KEYS[0],f_task_sensed);
	usleep(10000);
	f_task_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEYS[0]); /////////////////////////////////////////////////
	f_task_sensed -= force_bias_global;

	Matrix3d R_sensor = Matrix3d::Identity();
	robots[0]->rotation(R_sensor, "link7");
	f_task_sensed.head(3) += hand_mass * R_sensor.transpose() * Vector3d(0,0,-9.81);
	force_bias_adjustment = f_task_sensed;

	Matrix3d transformDev_Rob_palette = robot_pose_in_world[1].linear(); ///////////////////////////////////////////////
	auto haptic_controller_palette = new Sai2Primitives::OpenLoopTeleop(handler, 1, workspace_center_palette, posori_tasks[1]->_current_orientation, transformDev_Rob_palette);
	while(!haptic_controller_palette->device_started) 
	{
		cout << "starting right device" << endl;
	}
	haptic_controller_palette->GravityCompTask();
	//Task scaling factors
	double Ks_palette=2.0;
	double KsR_palette=1.0;
	haptic_controller_palette->setScalingFactors(Ks_palette, KsR_palette);
	
	bool gripper_state_palette = false;
	bool gripper_state_brush = false;
	bool previous_gripper_state_brush = false;
	// haptic_controller_palette->hapticDevice->enableForces(true); 
	// haptic_controller_brush->hapticDevice->enableForces(true); 
	haptic_controller_palette->EnableGripperUserSwitch();
	haptic_controller_brush->EnableGripperUserSwitch();

	// debug var
	Matrix3d debug_rot_init = Matrix3d::Identity();
	Matrix3d debug_rot_current = Matrix3d::Identity();

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
		// read force sensor data and remove bias and effecto from hand gravity
		f_task_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEYS[0]); /////////////////////////////////////////////////
		f_task_sensed -= force_bias_global; 
		// f_task_sensed -= force_bias_adjustment;

		Matrix3d R_sensor = Matrix3d::Identity();
		robots[0]->rotation(R_sensor, "link7");
		f_task_sensed.head(3) += hand_mass * R_sensor.transpose() * Vector3d(0,0,-9.81);
		f_task_sensed.tail(3) = Vector3d::Zero();
		f_task_sensed.head(3) = R_sensor*f_task_sensed.head(3);
		f_task_sensed = -f_task_sensed;

		// if(controller_counter % 100 == 0)
		// {
			// cout << "sensed forces\n" << f_task_sensed.transpose() << endl;
		// }

		// read renabling/disabling teleoperation brush
		remote_enabled = stoi(redis_client.get(REMOTE_ENABLED_KEY));
		// read restar cycle key
		restart_cycle = stoi(redis_client.get(RESTART_CYCLE_KEY));

///////////////////////////////////////////////////////////////////////////////////////////
		//// State Machine Robot 0 - Haptic Brush ////
		if(state_brush == GOTO_INITIAL_CONFIG)
		{
			// update robot home position task model
			N_prec[0].setIdentity();
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			// compute robot torques
			joint_tasks[0]->computeTorques(joint_task_torques[0]);
			command_torques[0] = joint_task_torques[0] + coriolis[0];
			
			// compute homing haptic device
			haptic_controller_brush->HomingTask();

			// read gripper state
			gripper_state_brush = haptic_controller_brush->ReadGripperUserSwitch();

			// cout << "left robot error : " << (joint_tasks[0]->_desired_position - joint_tasks[0]->_current_position).norm() << endl;

			if(remote_enabled==1 && (joint_tasks[0]->_desired_position - joint_tasks[0]->_current_position).norm() < 0.2 && haptic_controller_brush->device_homed && gripper_state_brush)
			{
				joint_tasks[0]->_ki = 0;
				posori_tasks[0]->reInitializeTask();
				workspace_center_brush = posori_tasks[0]->_current_position;
				haptic_center_brush = haptic_controller_brush->_pos_dev;

				haptic_controller_brush->setRobotCenter(workspace_center_brush, posori_tasks[0]->_current_orientation);
				haptic_controller_brush->setDeviceCenter(haptic_center_brush, haptic_controller_brush->_rot_dev);
				
				state_brush = HAPTIC_CONTROL;
			}
		}

		else if(state_brush == HAPTIC_CONTROL)
		{

			// update tasks model
			N_prec[0].setIdentity();
			posori_tasks[0]->updateTaskModel(N_prec[0]);
			N_prec[0] = posori_tasks[0]->_N;
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			
			// // read gripper state
			gripper_state_brush = haptic_controller_brush->ReadGripperUserSwitch();
			// compute haptic commands
			if(gripper_state_brush) //Full control
			{
				if(!previous_gripper_state_brush)
				{
					cMatrix3d rot_device_init;
					haptic_controller_brush->hapticDevice->getRotation(rot_device_init);
					haptic_controller_brush->setDeviceCenter(haptic_center_brush, rot_device_init.eigen());
					haptic_controller_brush->setRobotCenter(workspace_center_brush, posori_tasks[0]->_current_orientation);
				
				}
				Matrix3d desired_rotation_relative = Matrix3d::Identity();
				haptic_controller_brush->computeHapticCommands_ForceSensor( posori_tasks[0]->_desired_position,
																   posori_tasks[0]->_desired_orientation,
																   f_task_sensed, filter_on);

				cMatrix3d rot_device_current;
				haptic_controller_brush->hapticDevice->getRotation(rot_device_current);
				debug_rot_current = rot_device_current.eigen();

				// if(controller_counter % 100 == 0)
				// {
				// 	cout << "rotation rel\n" << debug_rot_current*debug_rot_init.transpose() << endl << endl;
				// 	cout << "rotation current\n" << debug_rot_current << endl << endl;
				// }


				//posori_tasks[0]->_desired_orientation = desired_rotation_relative * posori_tasks[0]->_cur_orientation;

			}
			else //Only position control
			{
				haptic_controller_brush->computeHapticCommands_ForceSensor_PositionOnly(posori_tasks[0]->_desired_position, f_task_sensed, filter_on);
			}

			// compute robot set torques
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			command_torques[0] = joint_task_torques[0] + coriolis[0] + posori_task_torques[0];


			if(remote_enabled == 0)
			{
			// set joint controller to maintin robot in current position
			joint_tasks[0]->reInitializeTask();
			// joint_tasks[0]->_desired_position = robot[0]->_q;
			// set current haptic device position
			haptic_controller_brush->setDeviceCenter(haptic_controller_brush->_pos_dev, haptic_controller_brush->_rot_dev);

			state_brush = MAINTAIN_POSITION;
			}
		}

		else if(state_brush == MAINTAIN_POSITION)
		{
			// update robot home position task model
			N_prec[0].setIdentity();
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			// compute robot torques
			joint_tasks[0]->computeTorques(joint_task_torques[0]);
			command_torques[0] = joint_task_torques[0] + coriolis[0];

			// compute homing haptic device
			haptic_controller_brush->HomingTask();

			// read gripper state
			gripper_state_brush = haptic_controller_brush->ReadGripperUserSwitch();

			if (remote_enabled==1 && gripper_state_brush)
			{
				posori_tasks[0]->reInitializeTask();

				haptic_controller_brush->setRobotCenter(workspace_center_brush, posori_tasks[0]->_current_orientation);
				haptic_controller_brush->setDeviceCenter(haptic_center_brush, haptic_controller_brush->_rot_dev);
				
				state_brush = HAPTIC_CONTROL;
			}
			else if (restart_cycle == 1)
			{
				// set joint controller to robot home position
				joint_tasks[0]->reInitializeTask();
				joint_tasks[0]->_desired_position = q_initial[0];
				// set haptic device home position
				haptic_controller_brush->setDeviceCenter(haptic_center_brush, haptic_controller_brush->_rot_dev);

				state_brush = GOTO_INITIAL_CONFIG;
			}
		}
		else
		{
			command_torques[0].setZero(dof[0]);
			haptic_controller_brush->GravityCompTask();
		}
		previous_gripper_state_brush = haptic_controller_brush->gripper_state;

///////////////////////////////////////////////////////////////////////////////////////////
		//// State Machine Robot 1 - Haptic Palette ////
		if(state_palette == GOTO_INITIAL_CONFIG)
		{
			// update robot home position task model
			N_prec[1].setIdentity();
			joint_tasks[1]->updateTaskModel(N_prec[1]);
			// compute robot torques
			joint_tasks[1]->computeTorques(joint_task_torques[1]);
			command_torques[1] = joint_task_torques[1] + coriolis[1];
			
			// compute homing haptic device
			haptic_controller_palette->HomingTask();

			// read gripper state
			gripper_state_palette = haptic_controller_palette->ReadGripperUserSwitch();

			if((joint_tasks[1]->_desired_position - joint_tasks[1]->_current_position).norm() < 0.2 && haptic_controller_palette->device_homed && gripper_state_palette)
			{
				joint_tasks[1]->_ki = 0;
				posori_tasks[1]->reInitializeTask();
				workspace_center_palette = posori_tasks[1]->_current_position;
				haptic_center_palette = haptic_controller_palette->_pos_dev;

				haptic_controller_palette->setRobotCenter(workspace_center_palette, posori_tasks[1]->_current_orientation);
				haptic_controller_palette->setDeviceCenter(haptic_center_palette, haptic_controller_palette->_rot_dev);
				
				state_palette = HAPTIC_CONTROL;
			}
		}

		else if(state_palette == HAPTIC_CONTROL)
		{

			// update tasks model
			N_prec[1].setIdentity();
			posori_tasks[1]->updateTaskModel(N_prec[1]);
			N_prec[1] = posori_tasks[1]->_N;
			joint_tasks[1]->updateTaskModel(N_prec[1]);

			//compute haptic commands (only position control) - without force feedback (force_sensed=0)
			haptic_controller_palette->computeHapticCommands_ForceSensor_PositionOnly(posori_tasks[1]->_desired_position);

			// compute robot set torques
			posori_tasks[1]->computeTorques(posori_task_torques[1]);
			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			command_torques[1] = joint_task_torques[1] + coriolis[1] + posori_task_torques[1];

			// read gripper state
			gripper_state_palette = haptic_controller_palette->ReadGripperUserSwitch();

			if(!gripper_state_palette)
			{
			// set joint controller to maintin robot in current position
			joint_tasks[1]->reInitializeTask();
			// joint_tasks[1]->_desired_position = robot[1]->_q;
			// set current haptic device position
			haptic_controller_palette->setDeviceCenter(haptic_controller_palette->_pos_dev, haptic_controller_palette->_rot_dev);

			state_palette = MAINTAIN_POSITION;
			}
		}

		else if(state_palette == MAINTAIN_POSITION)
		{
			// update robot home position task model
			N_prec[1].setIdentity();
			joint_tasks[1]->updateTaskModel(N_prec[1]);
			// compute robot torques
			joint_tasks[1]->computeTorques(joint_task_torques[1]);
			command_torques[1] = joint_task_torques[1] + coriolis[1];

			// compute homing haptic device
			haptic_controller_palette->HomingTask();

			// read gripper state
			gripper_state_palette = haptic_controller_palette->ReadGripperUserSwitch();

			if (gripper_state_palette)
			{
				posori_tasks[1]->reInitializeTask();

				haptic_controller_palette->setRobotCenter(workspace_center_palette, posori_tasks[1]->_current_orientation);
				haptic_controller_palette->setDeviceCenter(haptic_center_palette, haptic_controller_palette->_rot_dev);
				
				state_palette = HAPTIC_CONTROL;
			}
			else if (restart_cycle == 1)
			{
				// set joint controller to robot home position
				joint_tasks[1]->reInitializeTask();
				joint_tasks[1]->_desired_position = q_initial[1];
				// set haptic device home position
				haptic_controller_palette->setDeviceCenter(haptic_center_palette, haptic_controller_palette->_rot_dev);

				remote_enabled = 1;
				restart_cycle = 0;
				redis_client.set(REMOTE_ENABLED_KEY, to_string(remote_enabled));
				redis_client.set(RESTART_CYCLE_KEY, to_string(restart_cycle));

				state_palette = GOTO_INITIAL_CONFIG;
			}
		}
		else
		{
			command_torques[1].setZero(dof[1]);
			haptic_controller_brush->GravityCompTask();

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
	delete haptic_controller_palette;
	delete haptic_controller_brush;

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