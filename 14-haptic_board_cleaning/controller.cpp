// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "haptic_tasks/HapticController.h"
#include "haptic_tasks/BilateralPassivityController.h"

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
};

const int n_robots = robot_files.size();

// redis keys:
// - read:
vector<string> JOINT_ANGLES_KEYS  = {
	"sai2::WarehouseSimulation::panda1::sensors::q",
};
vector<string> JOINT_VELOCITIES_KEYS = {
	"sai2::WarehouseSimulation::panda1::sensors::dq",
};
vector<string> FORCE_SENSED_KEYS = {
	"sai2::WarehouseSimulation::panda1::sensors::f_op",
};

const string REMOTE_ENABLED_KEY = "sai2::WarehouseSimulation::sensors::remote_enabled";
const string RESTART_CYCLE_KEY = "sai2::WarehouseSimulation::sensors::restart_cycle";

// - write
vector<string> JOINT_TORQUES_COMMANDED_KEYS = {
	"sai2::WarehouseSimulation::panda1::actuators::fgc",
};

// - model
vector<string> MASSMATRIX_KEYS = 
{
	"sai2::WarehouseSimulation::panda1::model::mass_matrix",
};

vector<string> CORIOLIS_KEYS = 
{
	"sai2::WarehouseSimulation::panda1::model::coriolis",
};

vector<string> ROBOT_GRAVITY_KEYS = 
{
	"sai2::WarehouseSimulation::panda1::model::gravity",
};

//// Haptic device related keys ////
// Maximum stiffness, damping and force specifications
vector<string> DEVICE_MAX_STIFFNESS_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_stiffness",
};
vector<string> DEVICE_MAX_DAMPING_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_damping",
};
vector<string> DEVICE_MAX_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_force",
};
// Set force and torque feedback of the haptic device
vector<string> DEVICE_COMMANDED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force",
};
vector<string> DEVICE_COMMANDED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_torque",
};
vector<string> DEVICE_COMMANDED_GRIPPER_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper",
};
// Haptic device current position and rotation
vector<string> DEVICE_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position",
};
vector<string> DEVICE_ROTATION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rotation",
};
vector<string> DEVICE_GRIPPER_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position_gripper",
};
// Haptic device current velocity
vector<string> DEVICE_TRANS_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity",
};
vector<string> DEVICE_ROT_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity",
};
vector<string> DEVICE_GRIPPER_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity",
};
vector<string> DEVICE_SENSED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_force",
};
vector<string> DEVICE_SENSED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_torque",
};

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool inertia_regularization = true;

#define GOTO_INITIAL_CONFIG               0
#define HAPTIC_CONTROL                    1
#define MAINTAIN_POSITION				  2

int translation_counter = 0;

int remote_enabled = 1; //////////////////////////////////////////////////////// Read from redis ??
int restart_cycle = 0;

int state_eraser = GOTO_INITIAL_CONFIG;

unsigned long long controller_counter = 0;

RedisClient redis_client;

int main() {


	if(!flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEYS[0] = "sai2::FrankaPanda::Bonnie::actuators::fgc";
		JOINT_ANGLES_KEYS[0]  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEYS[0] = "sai2::FrankaPanda::Bonnie::sensors::dq";
		MASSMATRIX_KEYS[0] = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEYS[0] = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_GRAVITY_KEYS[0] = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";	
		FORCE_SENSED_KEYS[0] = "sai2::ATIGamma_Sensor::Bonnie::force_torque";
	}


	// position of robots in world
	vector<Affine3d> robot_pose_in_world;
	Affine3d pose = Affine3d::Identity();
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
	q_init_1 << -1.39968,-1.02909,1.04275,-1.84455,-0.357845,1.86703,-1.18647;

	q_initial.push_back(q_init_1);

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

		joint_tasks[i]->_desired_position = q_initial[i];
		joint_tasks[i]->_otg->setMaxVelocity(M_PI/6);

		// end effector tasks
		string link_name = "link7";
		Eigen::Vector3d pos_in_link = Vector3d(0.0,0.0,0.2);
		posori_tasks.push_back(new Sai2Primitives::PosOriTask(robots[i], link_name, pos_in_link));

		Affine3d sensor_frame = Affine3d::Identity();
		sensor_frame.translation() = Vector3d(0, 0, 0.12);
		posori_tasks[i]->setForceSensorFrame(link_name, sensor_frame);

		posori_task_torques.push_back(VectorXd::Zero(dof[i]));
		posori_tasks[i]->_use_interpolation_flag = false;
		posori_tasks[i]->_use_velocity_saturation_flag = true;
		posori_tasks[i]->_linear_saturation_velocity = 0.7;
		posori_tasks[i]->_angular_saturation_velocity = M_PI/1.5;

		posori_tasks[i]->_kp_pos = 200.0;
		posori_tasks[i]->_kv_pos = 15.0;
		posori_tasks[i]->_kp_ori = 200.0;
		posori_tasks[i]->_kv_ori = 15.0;		
	}


	// define home positions of the panda robots
	Vector3d workspace_center_eraser = posori_tasks[0]->_current_position;

	// define home position of the haptic device
	Vector3d haptic_center_eraser = Vector3d::Zero();

	////Haptic teleoperation controller ////
	//Robot Palette : Robot[0] && Robot Brush : Robot[1]
	auto eraser_teleop_task = new Sai2Primitives::HapticController(posori_tasks[0]->_current_position, posori_tasks[0]->_current_orientation, robot_pose_in_world[0].linear());

	eraser_teleop_task->_filter_on = true;
	eraser_teleop_task->setFilterCutOffFreq(0.004, 0.04);
	eraser_teleop_task->_haptic_feedback_from_proxy = false;
	eraser_teleop_task->_send_haptic_feedback = true;
	//Task scaling factors
	double Ks_eraser=2.0;
	double KsR_eraser=1.0;
	eraser_teleop_task->setScalingFactors(Ks_eraser, KsR_eraser);

	Matrix3d Red_factor_rot_eraser = Matrix3d::Identity();
	Matrix3d Red_factor_trans_eraser = Matrix3d::Identity();
	Red_factor_rot_eraser << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	Red_factor_trans_eraser << 1.0, 0.0, 0.0,
						  0.0, 1.0, 0.0,
						  0.0, 0.0, 1.0;
	eraser_teleop_task->setReductionFactorForceFeedback(Red_factor_trans_eraser, Red_factor_rot_eraser);
	
	VectorXd f_sensed_eraser = VectorXd::Zero(6);
	VectorXd force_bias_global_eraser = VectorXd::Zero(6);
	VectorXd force_bias_adjustment_eraser = VectorXd::Zero(6);
	double hand_eraser_mass = 0.0;
	Vector3d hand_eraser_com = Vector3d::Zero();
	
	if(!flag_simulation)
	{
		force_bias_adjustment_eraser << 0.3, 0.7, 0, 0, 0, 0;

		force_bias_global_eraser << -0.133241,   0.413947,    15.3625, -0.0922695, 0.00362847, 0.00647149;
		hand_eraser_mass = 1.56353;
		hand_eraser_com = Vector3d(-6.19239e-05,  -0.00452189,    0.0777715);
	}

	// Read haptic device specifications from haptic driver
	VectorXd _max_stiffness_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0]);
	VectorXd _max_damping_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0]);
	VectorXd _max_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0]);
	
	// set the device specifications to the haptic controller
	eraser_teleop_task->_max_linear_stiffness_device = _max_stiffness_device0[0];
	eraser_teleop_task->_max_angular_stiffness_device = _max_stiffness_device0[1];
	eraser_teleop_task->_max_linear_damping_device = _max_damping_device0[0];
	eraser_teleop_task->_max_angular_damping_device = _max_damping_device0[1];
	eraser_teleop_task->_max_force_device = _max_force_device0[0];
	eraser_teleop_task->_max_torque_device = _max_force_device0[1];

	bool gripper_state_eraser = false;
	bool previous_gripper_state_eraser = false;

	//// passivity observer and controller ////
	auto passivity_controller_eraser = new Sai2Primitives::BilateralPassivityController(posori_tasks[0], eraser_teleop_task);
	Vector3d haptic_damping_force_passivity_eraser = Vector3d::Zero();
	Vector3d command_force_device_plus_damping_eraser = Vector3d::Zero();

	// remove inertial forces from hand
	Vector3d hand_velocity = Vector3d::Zero();
	Vector3d prev_hand_velocity = Vector3d::Zero();
	Vector3d hand_acceleration = Vector3d::Zero();
	Vector3d hand_inertial_forces = Vector3d::Zero();

	// setup redis keys to be updated with the callback
	// objects to read from redis
	vector<MatrixXd> mass_from_robots;
	mass_from_robots.push_back(MatrixXd::Identity(7,7));

	vector<VectorXd> coriolis_from_robots;
	coriolis_from_robots.push_back(VectorXd::Zero(7));

	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEYS[0], robots[0]->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEYS[0], robots[0]->_dq);

	redis_client.addEigenToReadCallback(0, FORCE_SENSED_KEYS[0], f_sensed_eraser);

	redis_client.addEigenToReadCallback(0, DEVICE_POSITION_KEYS[0], eraser_teleop_task->_current_position_device);
	redis_client.addEigenToReadCallback(0, DEVICE_ROTATION_KEYS[0], eraser_teleop_task->_current_rotation_device);
	redis_client.addEigenToReadCallback(0, DEVICE_TRANS_VELOCITY_KEYS[0], eraser_teleop_task->_current_trans_velocity_device);
	redis_client.addEigenToReadCallback(0, DEVICE_ROT_VELOCITY_KEYS[0], eraser_teleop_task->_current_rot_velocity_device);
	redis_client.addEigenToReadCallback(0, DEVICE_SENSED_FORCE_KEYS[0], eraser_teleop_task->_sensed_force_device);
	redis_client.addEigenToReadCallback(0, DEVICE_SENSED_TORQUE_KEYS[0], eraser_teleop_task->_sensed_torque_device);
	redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_POSITION_KEYS[0], eraser_teleop_task->_current_position_gripper_device);
	redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_VELOCITY_KEYS[0], eraser_teleop_task->_current_gripper_velocity_device);

	redis_client.addIntToReadCallback(0, REMOTE_ENABLED_KEY, remote_enabled);
	redis_client.addIntToReadCallback(0, RESTART_CYCLE_KEY, restart_cycle);

	if(!flag_simulation)
	{
		redis_client.addEigenToReadCallback(0, MASSMATRIX_KEYS[0], mass_from_robots[0]);
		redis_client.addEigenToReadCallback(0, CORIOLIS_KEYS[0], coriolis_from_robots[0]);
	}

	// objects to write to redis
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEYS[0], command_torques[0]);
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_FORCE_KEYS[0], command_force_device_plus_damping_eraser);
	redis_client.addDoubleToWriteCallback(0, DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], eraser_teleop_task->_commanded_gripper_force_device);

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

		redis_client.executeReadCallback(0);

		// read robot state from redis and update robot model
		for(int i=0 ; i<n_robots ; i++)
		{
			// update model
			if(flag_simulation)
			{
				robots[i]->updateModel();
				robots[i]->coriolisForce(coriolis[i]);
			}
			else
			{
				robots[i]->updateKinematics();
				robots[i]->_M = mass_from_robots[i];
				if(inertia_regularization)
				{
					robots[i]->_M(4,4) += 0.07;
					robots[i]->_M(5,5) += 0.07;
					robots[i]->_M(6,6) += 0.07;
				}
				robots[i]->_M_inv = robots[i]->_M.inverse();
				coriolis[i] = coriolis_from_robots[i];
			}

		}

		// compute hand inertial forces
		hand_velocity = posori_tasks[0]->_current_velocity + posori_tasks[0]->_current_angular_velocity.cross(hand_eraser_com);
		if(controller_counter > 100)
		{
			hand_acceleration = (hand_velocity - prev_hand_velocity)/dt;
		}
		prev_hand_velocity = hand_velocity;
		hand_inertial_forces = hand_eraser_mass * hand_acceleration;


		// read force sensor data and remove bias and effecto from hand gravity
		f_sensed_eraser -= force_bias_global_eraser + force_bias_adjustment_eraser; 
		Matrix3d R_sensor_eraser = Matrix3d::Identity();
		robots[0]->rotation(R_sensor_eraser, "link7");
		Vector3d p_tool_sensorFrame_eraser = hand_eraser_mass * R_sensor_eraser.transpose() * Vector3d(0,0,-9.81); 
		f_sensed_eraser.head(3) += p_tool_sensorFrame_eraser;
		f_sensed_eraser.tail(3) += hand_eraser_com.cross(p_tool_sensorFrame_eraser);

		f_sensed_eraser.head(3) -= 0.8 * R_sensor_eraser.transpose() * hand_inertial_forces;

		posori_tasks[0]->updateSensedForceAndMoment(f_sensed_eraser.head(3), f_sensed_eraser.tail(3));
		VectorXd sensed_force_eraser_world_frame = VectorXd::Zero(6);
		sensed_force_eraser_world_frame << posori_tasks[0]->_sensed_force, posori_tasks[0]->_sensed_moment;
		eraser_teleop_task->updateSensedForce(-sensed_force_eraser_world_frame);

		// if(controller_counter % 100 == 0)
		// {
		// 	cout << "force brush : " << f_sensed_eraser.transpose() << endl;
		// 	cout << endl;
		// }

		eraser_teleop_task->UseGripperAsSwitch();

///////////////////////////////////////////////////////////////////////////////////////////
		//// State Machine Robot 0 - Haptic Eraser ////
		if(state_eraser == GOTO_INITIAL_CONFIG)
		{
			// update robot home position task model
			N_prec[0].setIdentity();
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			// compute robot torques
			joint_tasks[0]->computeTorques(joint_task_torques[0]);
			command_torques[0] = joint_task_torques[0] + coriolis[0];
			
			// compute homing haptic device
			eraser_teleop_task->HomingTask();

			// read gripper state
			gripper_state_eraser = eraser_teleop_task->gripper_state;

			if(remote_enabled==1 && (joint_tasks[0]->_desired_position - joint_tasks[0]->_current_position).norm() < 0.2 && eraser_teleop_task->device_homed && gripper_state_eraser)
			{
				joint_tasks[0]->_ki = 0;
				posori_tasks[0]->reInitializeTask();
				workspace_center_eraser = posori_tasks[0]->_current_position;
				haptic_center_eraser = eraser_teleop_task->_current_position_device;

				eraser_teleop_task->setRobotCenter(workspace_center_eraser, posori_tasks[0]->_current_orientation);
				eraser_teleop_task->setDeviceCenter(haptic_center_eraser, eraser_teleop_task->_current_rotation_device);
				
				state_eraser = HAPTIC_CONTROL;
			}
		}

		else if(state_eraser == HAPTIC_CONTROL)
		{

			// update tasks model
			N_prec[0].setIdentity();
			posori_tasks[0]->updateTaskModel(N_prec[0]);
			N_prec[0] = posori_tasks[0]->_N;
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			
			// // read gripper state
			gripper_state_eraser = eraser_teleop_task->gripper_state;
			// compute haptic commands
			if(gripper_state_eraser) //Full control
			{
				if(!previous_gripper_state_eraser)
				{
					eraser_teleop_task->setDeviceCenter(haptic_center_eraser, eraser_teleop_task->_current_rotation_device);
					eraser_teleop_task->setRobotCenter(workspace_center_eraser, posori_tasks[0]->_current_orientation);
				
				}
				Matrix3d desired_rotation_relative = Matrix3d::Identity();
				eraser_teleop_task->computeHapticCommands6d( posori_tasks[0]->_desired_position,
																   posori_tasks[0]->_desired_orientation);

			}
			else //Only position control
			{
				eraser_teleop_task->computeHapticCommands3d(posori_tasks[0]->_desired_position);
			}

			// compute robot set torques
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			command_torques[0] = joint_task_torques[0] + coriolis[0] + posori_task_torques[0];

			passivity_controller_eraser->computePOPCForce(haptic_damping_force_passivity_eraser);

			if(remote_enabled == 0)
			{
			// set joint controller to maintin robot in current position
			joint_tasks[0]->reInitializeTask();
			// joint_tasks[1]->_desired_position = robot[1]->_q;
			// set current haptic device position
			eraser_teleop_task->setDeviceCenter(eraser_teleop_task->_current_position_device, eraser_teleop_task->_current_rotation_device);

			state_eraser = MAINTAIN_POSITION;
			}
		}

		else if(state_eraser == MAINTAIN_POSITION)
		{
			// update robot home position task model
			N_prec[0].setIdentity();
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			// compute robot torques
			joint_tasks[0]->computeTorques(joint_task_torques[0]);
			command_torques[0] = joint_task_torques[0] + coriolis[0];

			// compute homing haptic device
			eraser_teleop_task->HomingTask();

			// read gripper state
			gripper_state_eraser = eraser_teleop_task->gripper_state;



			if (remote_enabled==1 && gripper_state_eraser)
			{
				posori_tasks[0]->reInitializeTask();

				eraser_teleop_task->setRobotCenter(workspace_center_eraser, posori_tasks[0]->_current_orientation);
				eraser_teleop_task->setDeviceCenter(haptic_center_eraser, eraser_teleop_task->_current_rotation_device);
				
				state_eraser = HAPTIC_CONTROL;
			}
			else if (restart_cycle == 1)
			{
				// set joint controller to robot home position
				joint_tasks[0]->reInitializeTask();
				joint_tasks[0]->_desired_position = q_initial[0];
				// set haptic device home position
				eraser_teleop_task->setDeviceCenter(haptic_center_eraser, eraser_teleop_task->_current_rotation_device);

				state_eraser = GOTO_INITIAL_CONFIG;
			}
		}
		else
		{
			command_torques[0].setZero(dof[0]);
			eraser_teleop_task->GravityCompTask();
		}
		previous_gripper_state_eraser = eraser_teleop_task->gripper_state;


		

		// send to redis
		command_force_device_plus_damping_eraser = eraser_teleop_task->_commanded_force_device + haptic_damping_force_passivity_eraser;
		
		// 
		redis_client.executeWriteCallback(0);

		prev_time = current_time;
		controller_counter++;
	}



	for(int i=0 ; i<n_robots ; i++)
	{
		redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[i], Vector3d::Zero());
		redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[i], Vector3d::Zero());
		redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[i], "0.0");

		command_torques[i].setZero();
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
