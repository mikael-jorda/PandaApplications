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
	"./resources/panda_arm_palette.urdf",
	"./resources/panda_arm_brush.urdf",
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

//// Haptic device related keys ////
// Maximum stiffness, damping and force specifications
vector<string> DEVICE_MAX_STIFFNESS_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_stiffness",
	"sai2::ChaiHapticDevice::device1::specifications::max_stiffness",
};
vector<string> DEVICE_MAX_DAMPING_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_damping",
	"sai2::ChaiHapticDevice::device1::specifications::max_damping",
};
vector<string> DEVICE_MAX_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_force",
	"sai2::ChaiHapticDevice::device1::specifications::max_force",
};
// Set force and torque feedback of the haptic device
vector<string> DEVICE_COMMANDED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force",
};
vector<string> DEVICE_COMMANDED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_torque",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_torque",
};
vector<string> DEVICE_COMMANDED_GRIPPER_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force_gripper",
};
// Haptic device current position and rotation
vector<string> DEVICE_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position",
	"sai2::ChaiHapticDevice::device1::sensors::current_position",
};
vector<string> DEVICE_ROTATION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rotation",
	"sai2::ChaiHapticDevice::device1::sensors::current_rotation",
};
vector<string> DEVICE_GRIPPER_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position_gripper",
	"sai2::ChaiHapticDevice::device1::sensors::current_position_gripper",
};
// Haptic device current velocity
vector<string> DEVICE_TRANS_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_trans_velocity",
};
vector<string> DEVICE_ROT_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_rot_velocity",
};
vector<string> DEVICE_GRIPPER_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_gripper_velocity",
};
vector<string> DEVICE_SENSED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_force",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_force",
};
vector<string> DEVICE_SENSED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_torque",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_torque",
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
		FORCE_SENSED_KEYS[0] = "sai2::ATIGamma_Sensor::Clyde::force_torque";

		JOINT_TORQUES_COMMANDED_KEYS[1] = "sai2::FrankaPanda::Bonnie::actuators::fgc";
		JOINT_ANGLES_KEYS[1]  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::dq";
		MASSMATRIX_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_GRAVITY_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";	
		FORCE_SENSED_KEYS[1] = "sai2::ATIGamma_Sensor::Bonnie::force_torque";
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
	q_init_1 << 1.58126,-1.32543,-1.109,-2.52754,0.905724,1.84656,-1.12792;
	q_init_2 << -1.49122,-1.00293,0.913876,-1.99563,-0.55207,2.4641,1.33142;

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

	VectorXd f_sensed_brush = VectorXd::Zero(6);
	VectorXd f_sensed_palette = VectorXd::Zero(6);

	// define home positions of the panda robots
	Vector3d workspace_center_palette = posori_tasks[0]->_current_position;
	Vector3d workspace_center_brush = posori_tasks[1]->_current_position;

	// define home position of the haptic device
	Vector3d haptic_center_palette = Vector3d::Zero();
	Vector3d haptic_center_brush = Vector3d::Zero();

	////Haptic teleoperation controller ////
	//Robot Palette : Robot[0] && Robot Brush : Robot[1]
	auto brush_teleop_task = new Sai2Primitives::HapticController(posori_tasks[1]->_current_position, posori_tasks[1]->_current_orientation, robot_pose_in_world[1].linear());

	brush_teleop_task->_filter_on = true;
	brush_teleop_task->setFilterCutOffFreq(0.007, 0.04);
	brush_teleop_task->_haptic_feedback_from_proxy = false;
	brush_teleop_task->_send_haptic_feedback = true;
	//Task scaling factors
	double Ks_brush=2.0;
	double KsR_brush=1.0;
	brush_teleop_task->setScalingFactors(Ks_brush, KsR_brush);

	Matrix3d Red_factor_rot_brush = Matrix3d::Identity();
	Matrix3d Red_factor_trans_brush = Matrix3d::Identity();
	Red_factor_rot_brush << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	Red_factor_trans_brush << 1.1, 0.0, 0.0,
						  0.0, 1.1, 0.0,
						  0.0, 0.0, 1.1;
	brush_teleop_task->setReductionFactorForceFeedback(Red_factor_trans_brush, Red_factor_rot_brush);
	
	VectorXd f_task_sensed_brush = VectorXd::Zero(6);
	VectorXd force_bias_global_brush = VectorXd::Zero(6);
	double hand_brush_mass = 0.0;
	Vector3d hand_brush_com = Vector3d::Zero();
	
	if(!flag_simulation)
	{
		// -15.0335    2.36657   0.996593  -0.202162    -1.1767 -0.0399846
		// 1.5546
		// 9.21258e-06  -0.0041225   0.0777601
		force_bias_global_brush << -15.0277,     2.3619,    1.11118,  -0.202308,   -1.17481, -0.0457088;
		hand_brush_mass = 1.55647;
		hand_brush_com = Vector3d(-7.30332e-05,  -0.00447694,    0.0777536);
	}

	auto palette_teleop_task = new Sai2Primitives::HapticController(posori_tasks[0]->_current_position, posori_tasks[0]->_current_orientation, robot_pose_in_world[0].linear());
	// palette_teleop_task->_filter_on = true;
	// palette_teleop_task->setFilterCutOffFreq(0.04, 0.04);
	palette_teleop_task->_haptic_feedback_from_proxy = false;
	palette_teleop_task->_send_haptic_feedback = false;

	//Task scaling factors
	double Ks_palette=2.0;
	double KsR_palette=1.0;
	palette_teleop_task->setScalingFactors(Ks_palette, KsR_palette);

	Matrix3d Red_factor_rot_palette = Matrix3d::Identity();
	Matrix3d Red_factor_trans_palette = Matrix3d::Identity();
	Red_factor_rot_palette << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

	Red_factor_trans_palette << 1.0, 0.0, 0.0,
						  0.0, 1.0, 0.0,
						  0.0, 0.0, 1.0;
	palette_teleop_task->setReductionFactorForceFeedback(Red_factor_trans_palette, Red_factor_rot_palette);
	
	VectorXd f_task_sensed_palette = VectorXd::Zero(6);
	VectorXd force_bias_global_palette = VectorXd::Zero(6);
	double hand_palette_mass = 0.0;
	Vector3d hand_palette_com = Vector3d::Zero();
	
	// if(!flag_simulation)
	// {
	// 	force_bias_global_palette << -2.32819,  -3.88484,   -129.07,  -0.38751,   1.57918, -0.021328;
	// 	hand_palette_mass = 1.5;
	// 	hand_palette_com = Vector3d(0.0, 0.0, 0.1);
	// }

	// Read haptic device specifications from haptic driver
	VectorXd _max_stiffness_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0]);
	VectorXd _max_damping_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0]);
	VectorXd _max_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0]);
	VectorXd _max_stiffness_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[1]);
	VectorXd _max_damping_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[1]);
	VectorXd _max_force_device1 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[1]);
	
	// set the device specifications to the haptic controller
	palette_teleop_task->_max_linear_stiffness_device = _max_stiffness_device0[0];
	palette_teleop_task->_max_angular_stiffness_device = _max_stiffness_device0[1];
	palette_teleop_task->_max_linear_damping_device = _max_damping_device0[0];
	palette_teleop_task->_max_angular_damping_device = _max_damping_device0[1];
	palette_teleop_task->_max_force_device = _max_force_device0[0];
	palette_teleop_task->_max_torque_device = _max_force_device0[1];

	brush_teleop_task->_max_linear_stiffness_device = _max_stiffness_device1[0];
	brush_teleop_task->_max_angular_stiffness_device = _max_stiffness_device1[1];
	brush_teleop_task->_max_linear_damping_device = _max_damping_device1[0];
	brush_teleop_task->_max_angular_damping_device = _max_damping_device1[1];
	brush_teleop_task->_max_force_device = _max_force_device1[0];
	brush_teleop_task->_max_torque_device = _max_force_device1[1];

	bool gripper_state_palette = false;
	bool gripper_state_brush = false;
	bool previous_gripper_state_brush = false;

	//// passivity observer and controller ////
	auto passivity_controller_brush = new Sai2Primitives::BilateralPassivityController(posori_tasks[1], brush_teleop_task);
	Vector3d haptic_damping_force_passivity_brush = Vector3d::Zero();
	Vector3d command_force_device_plus_damping_brush = Vector3d::Zero();

	Vector3d command_force_device_plus_damping_palette = Vector3d::Zero();

	// remove inertial forces from hand
	Vector3d hand_velocity = Vector3d::Zero();
	Vector3d prev_hand_velocity = Vector3d::Zero();
	Vector3d hand_acceleration = Vector3d::Zero();
	Vector3d hand_inertial_forces = Vector3d::Zero();

	// setup redis keys to be updated with the callback
	// objects to read from redis
	vector<MatrixXd> mass_from_robots;
	mass_from_robots.push_back(MatrixXd::Identity(7,7));
	mass_from_robots.push_back(MatrixXd::Identity(7,7));

	vector<VectorXd> coriolis_from_robots;
	coriolis_from_robots.push_back(VectorXd::Zero(7));
	coriolis_from_robots.push_back(VectorXd::Zero(7));

	redis_client.addEigenToRead(JOINT_ANGLES_KEYS[0], robots[0]->_q);
	redis_client.addEigenToRead(JOINT_VELOCITIES_KEYS[0], robots[0]->_dq);
	redis_client.addEigenToRead(JOINT_ANGLES_KEYS[1], robots[1]->_q);
	redis_client.addEigenToRead(JOINT_VELOCITIES_KEYS[1], robots[1]->_dq);

	// redis_client.addEigenToRead(FORCE_SENSED_KEYS[0], f_sensed_palette);
	redis_client.addEigenToRead(FORCE_SENSED_KEYS[1], f_sensed_brush);

	redis_client.addEigenToRead(DEVICE_POSITION_KEYS[0], palette_teleop_task->_current_position_device);
	redis_client.addEigenToRead(DEVICE_ROTATION_KEYS[0], palette_teleop_task->_current_rotation_device);
	redis_client.addEigenToRead(DEVICE_TRANS_VELOCITY_KEYS[0], palette_teleop_task->_current_trans_velocity_device);
	redis_client.addEigenToRead(DEVICE_ROT_VELOCITY_KEYS[0], palette_teleop_task->_current_rot_velocity_device);
	redis_client.addEigenToRead(DEVICE_SENSED_FORCE_KEYS[0], palette_teleop_task->_sensed_force_device);
	redis_client.addEigenToRead(DEVICE_SENSED_TORQUE_KEYS[0], palette_teleop_task->_sensed_torque_device);
	redis_client.addDoubleToRead(DEVICE_GRIPPER_POSITION_KEYS[0], palette_teleop_task->_current_position_gripper_device);
	redis_client.addDoubleToRead(DEVICE_GRIPPER_VELOCITY_KEYS[0], palette_teleop_task->_current_gripper_velocity_device);

	redis_client.addEigenToRead(DEVICE_POSITION_KEYS[1], brush_teleop_task->_current_position_device);
	redis_client.addEigenToRead(DEVICE_ROTATION_KEYS[1], brush_teleop_task->_current_rotation_device);
	redis_client.addEigenToRead(DEVICE_TRANS_VELOCITY_KEYS[1], brush_teleop_task->_current_trans_velocity_device);
	redis_client.addEigenToRead(DEVICE_ROT_VELOCITY_KEYS[1], brush_teleop_task->_current_rot_velocity_device);
	redis_client.addEigenToRead(DEVICE_SENSED_FORCE_KEYS[1], brush_teleop_task->_sensed_force_device);
	redis_client.addEigenToRead(DEVICE_SENSED_TORQUE_KEYS[1], brush_teleop_task->_sensed_torque_device);
	redis_client.addDoubleToRead(DEVICE_GRIPPER_POSITION_KEYS[1], brush_teleop_task->_current_position_gripper_device);
	redis_client.addDoubleToRead(DEVICE_GRIPPER_VELOCITY_KEYS[1], brush_teleop_task->_current_gripper_velocity_device);

	redis_client.addIntToRead(REMOTE_ENABLED_KEY, remote_enabled);
	redis_client.addIntToRead(RESTART_CYCLE_KEY, restart_cycle);

	if(!flag_simulation)
	{
		redis_client.addEigenToRead(MASSMATRIX_KEYS[0], mass_from_robots[0]);
		redis_client.addEigenToRead(MASSMATRIX_KEYS[1], mass_from_robots[1]);
		redis_client.addEigenToRead(CORIOLIS_KEYS[0], coriolis_from_robots[0]);
		redis_client.addEigenToRead(CORIOLIS_KEYS[1], coriolis_from_robots[1]);
	}

	// objects to write to redis
	redis_client.addEigenToWrite(JOINT_TORQUES_COMMANDED_KEYS[0], command_torques[0]);
	redis_client.addEigenToWrite(DEVICE_COMMANDED_FORCE_KEYS[0], command_force_device_plus_damping_palette);
	redis_client.addDoubleToWrite(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], palette_teleop_task->_commanded_gripper_force_device);

	redis_client.addEigenToWrite(JOINT_TORQUES_COMMANDED_KEYS[1], command_torques[1]);
	redis_client.addEigenToWrite(DEVICE_COMMANDED_FORCE_KEYS[1], command_force_device_plus_damping_brush);
	redis_client.addDoubleToWrite(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[1], brush_teleop_task->_commanded_gripper_force_device);

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

		redis_client.readAllSetupValues();

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
		hand_velocity = posori_tasks[1]->_current_velocity + posori_tasks[1]->_current_angular_velocity.cross(hand_brush_com);
		if(controller_counter > 100)
		{
			hand_acceleration = (hand_velocity - prev_hand_velocity)/dt;
		}
		prev_hand_velocity = hand_velocity;
		hand_inertial_forces = hand_brush_mass * hand_acceleration;


		// read force sensor data and remove bias and effecto from hand gravity
		f_sensed_brush -= force_bias_global_brush; 
		Matrix3d R_sensor_brush = Matrix3d::Identity();
		robots[1]->rotation(R_sensor_brush, "link7");
		Vector3d p_tool_sensorFrame_brush = hand_brush_mass * R_sensor_brush.transpose() * Vector3d(0,0,-9.81); 
		f_sensed_brush.head(3) += p_tool_sensorFrame_brush;
		f_sensed_brush.tail(3) += hand_brush_com.cross(p_tool_sensorFrame_brush);

		f_sensed_brush.head(3) -= 0.9 * R_sensor_brush.transpose() * hand_inertial_forces;

		posori_tasks[1]->updateSensedForceAndMoment(f_sensed_brush.head(3), f_sensed_brush.tail(3));
		VectorXd sensed_force_brush_world_frame = VectorXd::Zero(6);
		sensed_force_brush_world_frame << posori_tasks[1]->_sensed_force, posori_tasks[1]->_sensed_moment;
		brush_teleop_task->updateSensedForce(-sensed_force_brush_world_frame);

		// f_sensed_palette -= force_bias_global_palette; 
		// Matrix3d R_sensor_palette = Matrix3d::Identity();
		// robots[0]->rotation(R_sensor_palette, "link7");
		// Vector3d p_tool_sensorFrame_palette = hand_palette_mass * R_sensor_palette.transpose() * Vector3d(0,0,-9.81); 
		// f_sensed_palette.head(3) += p_tool_sensorFrame_palette;
		// f_sensed_palette.tail(3) += hand_palette_com.cross(p_tool_sensorFrame_palette);
		// posori_tasks[0]->updateSensedForceAndMoment(f_sensed_palette.head(3), f_sensed_palette.tail(3));
		// VectorXd sensed_force_palette_world_frame = VectorXd::Zero(6);
		// sensed_force_palette_world_frame << posori_tasks[0]->_sensed_force, posori_tasks[0]->_sensed_moment;
		// palette_teleop_task->updateSensedForce(-sensed_force_palette_world_frame);

		// if(controller_counter % 100 == 0)
		// {
		// 	cout << "force brush : " << f_sensed_brush.transpose() << endl;
		// 	cout << "force palette : " << f_sensed_palette.transpose() << endl;
		// 	cout << endl;
		// }

		palette_teleop_task->UseGripperAsSwitch();
		brush_teleop_task->UseGripperAsSwitch();

///////////////////////////////////////////////////////////////////////////////////////////
		//// State Machine Robot 1 - Haptic Brush ////
		if(state_brush == GOTO_INITIAL_CONFIG)
		{
			// update robot home position task model
			N_prec[1].setIdentity();
			joint_tasks[1]->updateTaskModel(N_prec[1]);
			// compute robot torques
			joint_tasks[1]->computeTorques(joint_task_torques[1]);
			command_torques[1] = joint_task_torques[1] + coriolis[1];
			
			// compute homing haptic device
			brush_teleop_task->HomingTask();

			// read gripper state
			gripper_state_brush = brush_teleop_task->gripper_state;

			if(remote_enabled==1 && (joint_tasks[1]->_desired_position - joint_tasks[1]->_current_position).norm() < 0.2 && brush_teleop_task->device_homed && gripper_state_brush)
			{
				joint_tasks[1]->_ki = 0;
				posori_tasks[1]->reInitializeTask();
				workspace_center_brush = posori_tasks[1]->_current_position;
				haptic_center_brush = brush_teleop_task->_current_position_device;

				brush_teleop_task->setRobotCenter(workspace_center_brush, posori_tasks[1]->_current_orientation);
				brush_teleop_task->setDeviceCenter(haptic_center_brush, brush_teleop_task->_current_rotation_device);
				
				state_brush = HAPTIC_CONTROL;
			}
		}

		else if(state_brush == HAPTIC_CONTROL)
		{

			// update tasks model
			N_prec[1].setIdentity();
			posori_tasks[1]->updateTaskModel(N_prec[1]);
			N_prec[1] = posori_tasks[1]->_N;
			joint_tasks[1]->updateTaskModel(N_prec[1]);
			
			// // read gripper state
			gripper_state_brush = brush_teleop_task->gripper_state;
			// compute haptic commands
			if(gripper_state_brush) //Full control
			{
				if(!previous_gripper_state_brush)
				{
					brush_teleop_task->setDeviceCenter(haptic_center_brush, brush_teleop_task->_current_rotation_device);
					brush_teleop_task->setRobotCenter(workspace_center_brush, posori_tasks[1]->_current_orientation);
				
				}
				Matrix3d desired_rotation_relative = Matrix3d::Identity();
				brush_teleop_task->computeHapticCommands6d( posori_tasks[1]->_desired_position,
																   posori_tasks[1]->_desired_orientation);

			}
			else //Only position control
			{
				brush_teleop_task->computeHapticCommands3d(posori_tasks[1]->_desired_position);
			}

			// compute robot set torques
			posori_tasks[1]->computeTorques(posori_task_torques[1]);
			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			command_torques[1] = joint_task_torques[1] + coriolis[1] + posori_task_torques[1];


			if(remote_enabled == 0)
			{
			// set joint controller to maintin robot in current position
			joint_tasks[1]->reInitializeTask();
			// joint_tasks[1]->_desired_position = robot[1]->_q;
			// set current haptic device position
			brush_teleop_task->setDeviceCenter(brush_teleop_task->_current_position_device, brush_teleop_task->_current_rotation_device);

			state_brush = MAINTAIN_POSITION;
			}
		}

		else if(state_brush == MAINTAIN_POSITION)
		{
			// update robot home position task model
			N_prec[1].setIdentity();
			joint_tasks[1]->updateTaskModel(N_prec[1]);
			// compute robot torques
			joint_tasks[1]->computeTorques(joint_task_torques[1]);
			command_torques[1] = joint_task_torques[1] + coriolis[1];

			// compute homing haptic device
			brush_teleop_task->HomingTask();

			// read gripper state
			gripper_state_brush = brush_teleop_task->gripper_state;

			if (remote_enabled==1 && gripper_state_brush)
			{
				posori_tasks[1]->reInitializeTask();

				brush_teleop_task->setRobotCenter(workspace_center_brush, posori_tasks[1]->_current_orientation);
				brush_teleop_task->setDeviceCenter(haptic_center_brush, brush_teleop_task->_current_rotation_device);
				
				state_brush = HAPTIC_CONTROL;
			}
			else if (restart_cycle == 1)
			{
				// set joint controller to robot home position
				joint_tasks[1]->reInitializeTask();
				joint_tasks[1]->_desired_position = q_initial[1];
				// set haptic device home position
				brush_teleop_task->setDeviceCenter(haptic_center_brush, brush_teleop_task->_current_rotation_device);

				state_brush = GOTO_INITIAL_CONFIG;
			}
		}
		else
		{
			command_torques[1].setZero(dof[1]);
			brush_teleop_task->GravityCompTask();
		}
		previous_gripper_state_brush = brush_teleop_task->gripper_state;

///////////////////////////////////////////////////////////////////////////////////////////
		//// State Machine Robot 0 - Haptic Palette ////
		if(state_palette == GOTO_INITIAL_CONFIG)
		{
			// update robot home position task model
			N_prec[0].setIdentity();
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			// compute robot torques
			joint_tasks[0]->computeTorques(joint_task_torques[0]);
			command_torques[0] = joint_task_torques[0] + coriolis[0];
			
			// compute homing haptic device
			palette_teleop_task->HomingTask();

			// read gripper state
			gripper_state_palette = palette_teleop_task->gripper_state;

			if((joint_tasks[0]->_desired_position - joint_tasks[0]->_current_position).norm() < 0.2 && palette_teleop_task->device_homed && gripper_state_palette)
			{
				joint_tasks[0]->_ki = 0;
				posori_tasks[0]->reInitializeTask();
				workspace_center_palette = posori_tasks[0]->_current_position;
				haptic_center_palette = palette_teleop_task->_current_position_device;

				passivity_controller_brush->reInitializeTask();

				palette_teleop_task->setRobotCenter(workspace_center_palette, posori_tasks[0]->_current_orientation);
				palette_teleop_task->setDeviceCenter(haptic_center_palette, palette_teleop_task->_current_rotation_device);
				
				state_palette = HAPTIC_CONTROL;
			}
		}

		else if(state_palette == HAPTIC_CONTROL)
		{

			// update tasks model
			N_prec[0].setIdentity();
			posori_tasks[0]->updateTaskModel(N_prec[0]);
			N_prec[0] = posori_tasks[0]->_N;
			joint_tasks[0]->updateTaskModel(N_prec[0]);

			//compute haptic commands (only position control) - without force feedback (force_sensed=0)
			palette_teleop_task->computeHapticCommands3d(posori_tasks[0]->_desired_position);

			// compute robot set torques
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			command_torques[0] = joint_task_torques[0] + coriolis[0] + posori_task_torques[0];

			passivity_controller_brush->computePOPCForce(haptic_damping_force_passivity_brush);

			// read gripper state
			gripper_state_palette = palette_teleop_task->gripper_state;

			if(!gripper_state_palette)
			{
			// set joint controller to maintin robot in current position
			joint_tasks[0]->reInitializeTask();
			// joint_tasks[0]->_desired_position = robot[0]->_q;
			// set current haptic device position
			palette_teleop_task->setDeviceCenter(palette_teleop_task->_current_position_device, palette_teleop_task->_current_rotation_device);

			state_palette = MAINTAIN_POSITION;
			}
		}

		else if(state_palette == MAINTAIN_POSITION)
		{
			// update robot home position task model
			N_prec[0].setIdentity();
			joint_tasks[0]->updateTaskModel(N_prec[0]);
			// compute robot torques
			joint_tasks[0]->computeTorques(joint_task_torques[0]);
			command_torques[0] = joint_task_torques[0] + coriolis[0];

			// compute homing haptic device
			palette_teleop_task->HomingTask();

			// read gripper state
			gripper_state_palette = palette_teleop_task->gripper_state;

			if (gripper_state_palette)
			{
				posori_tasks[0]->reInitializeTask();

				palette_teleop_task->setRobotCenter(workspace_center_palette, posori_tasks[0]->_current_orientation);
				palette_teleop_task->setDeviceCenter(haptic_center_palette, palette_teleop_task->_current_rotation_device);
				
				state_palette = HAPTIC_CONTROL;
			}
			else if (restart_cycle == 1)
			{
				// set joint controller to robot home position
				joint_tasks[0]->reInitializeTask();
				joint_tasks[0]->_desired_position = q_initial[0];
				// set haptic device home position
				palette_teleop_task->setDeviceCenter(haptic_center_palette, palette_teleop_task->_current_rotation_device);

				remote_enabled = 1;
				restart_cycle = 0;
				redis_client.set(REMOTE_ENABLED_KEY, to_string(remote_enabled));
				redis_client.set(RESTART_CYCLE_KEY, to_string(restart_cycle));

				state_palette = GOTO_INITIAL_CONFIG;
			}
		}
		else
		{
			command_torques[0].setZero(dof[0]);
			brush_teleop_task->GravityCompTask();

		}
///////////////////////////////////////////////////////////////////////////////////////////

		

		// send to redis
		command_force_device_plus_damping_brush = brush_teleop_task->_commanded_force_device + haptic_damping_force_passivity_brush;
		command_force_device_plus_damping_palette = palette_teleop_task->_commanded_force_device;
		
		if(controller_counter % 100 == 0)
		{
			cout << -sensed_force_brush_world_frame.transpose() << endl;
			cout << endl;
		}
		// 
		// command_force_device_plus_damping_brush.setZero();
		// command_force_device_plus_damping_palette.setZero();
		redis_client.writeAllSetupValues();

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