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

// - allegro hand
const string ALLGERO_CONTROL_MODE_KEY = "sai2::allegroHand::controller::control_mode";
const string ALLGERO_TORQUE_COMMANDED_KEY = "sai2::allegroHand::controller::joint_torques_commanded";
const string ALLGERO_POSITION_COMMANDED_KEY = "sai2::allegroHand::controller::joint_positions_commanded";
const string ALLGERO_PALM_ORIENTATION_KEY = "sai2::allegroHand::controller::palm_orientation";


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

vector<int> state = {
	GOTO_INITIAL_CONFIG,
	GOTO_INITIAL_CONFIG,
};

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
	vector<VectorXd> f_sensed;

	vector<VectorXd> q_initial;
	VectorXd q_init_1 = VectorXd::Zero(7);
	VectorXd q_init_2 = VectorXd::Zero(7);
	q_init_1 << -0.295513,0.96967,1.54262,-1.72179,0.330342,1.18991,-0.131168;
	q_init_2 << 0.407303,-0.831855,-0.851687,-1.86625,-0.0416269,1.35917,0.385143;

	q_initial.push_back(q_init_1);
	q_initial.push_back(q_init_2);

	vector<string> link_names =
	{
		"link7",
		"link7",
	};
	vector<Vector3d> pos_in_link = 
	{
		Vector3d(0.0, 0.0, 0.205),
		Vector3d(0.150, 0.0, 0.157),
	};

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
		joint_tasks[i]->_ki = 50.0;

		joint_tasks[i]->_desired_position = q_initial[i];
		joint_tasks[i]->_otg->setMaxVelocity(M_PI/6);

		// end effector tasks
		posori_tasks.push_back(new Sai2Primitives::PosOriTask(robots[i], link_names[i], pos_in_link[i]));

		Affine3d sensor_frame = Affine3d::Identity();
		sensor_frame.translation() = Vector3d(0, 0, 0.12);
		posori_tasks[i]->setForceSensorFrame(link_names[i], sensor_frame);

		posori_task_torques.push_back(VectorXd::Zero(dof[i]));
		posori_tasks[i]->_use_interpolation_flag = false;
		posori_tasks[i]->_use_velocity_saturation_flag = true;
		posori_tasks[i]->_linear_saturation_velocity = 0.7;
		posori_tasks[i]->_angular_saturation_velocity = M_PI/1.5;

		posori_tasks[i]->_kp_pos = 200.0;
		posori_tasks[i]->_kv_pos = 15.0;
		posori_tasks[i]->_kp_ori = 200.0;
		posori_tasks[i]->_kv_ori = 15.0;	

		f_sensed.push_back(VectorXd::Zero(6));	
	}


	////Haptic teleoperation controller ////
	//Left hand with allegro : Robot[0] && Right hand with wrench : Robot[1]
	// teleoperation tasks
	vector<Vector3d> workspace_centers;
	vector<Vector3d> haptic_centers;

	vector<Sai2Primitives::HapticController*> teleop_tasks;

	vector<bool> gripper_state;
	vector<bool> previous_gripper_state;

	// hand configuration
	VectorXd allegro_open_config = VectorXd::Zero(16);
	VectorXd allegro_closed_config = VectorXd::Zero(16);
	VectorXd allegro_desired_config = VectorXd::Zero(16);

	allegro_open_config << -0.0176639,0.284131,0.7923,0.147169,0.127286,0.2512,0.760434,0.415234,-0.0713656,0.188888,0.693773,0.6509,-0.705312,-0.144595,0.189509,0.353899;
	// allegro_closed_config << 0.0649746,1.50365,1.49193,1.39385,0.0964855,1.53418,1.61638,1.14842,0.313245,1.46291,1.5838,1.60901,-0.57989,-0.0799756,0.958731,1.06347;
	allegro_closed_config << -0.0624893,1.45847,1.56605,1.73612,0.155602,1.4242,1.67097,1.71695,0.395706,1.31929,1.70159,1.75645,-0.580866,-0.0797981,0.958642,1.08673;

	allegro_desired_config = allegro_open_config;
	redis_client.set(ALLGERO_CONTROL_MODE_KEY, "p");
	redis_client.setEigenMatrixJSON(ALLGERO_POSITION_COMMANDED_KEY, allegro_desired_config);
	// redis_client.setEigenMatrixJSON(ALLGERO_POSITION_COMMANDED_KEY, VectorXd::Zero(16));

	// hand palm orientation
	Matrix3d R_ee_palm = Matrix3d::Identity();
	Matrix3d R_world_ee = Matrix3d::Identity();
	Matrix3d R_world_palm = Matrix3d::Identity();
	R_ee_palm << 0, 0, 1,
				0, -1, 0,
				1, 0, 0;

	robots[0]->rotation(R_world_ee, link_names[0]);
	R_world_palm = R_world_ee * R_ee_palm;

	redis_client.setEigenMatrixJSON(ALLGERO_PALM_ORIENTATION_KEY, R_world_palm);

	bool close_gripper = true;

	// passivity
	vector<Sai2Primitives::BilateralPassivityController*> passivity_controllers;
	vector<Vector3d> passivity_damping_force;
	vector<Vector3d> haptic_force_plus_passivity;

	// ee_inertial_forces
	vector<Vector3d> ee_velocity;
	vector<Vector3d> prev_ee_velocity;
	vector<Vector3d> ee_acceleration;
	vector<Vector3d> ee_inertial_forces;	

	for(int i=0 ; i<n_robots ; i++)
	{
		// haptic task
		workspace_centers.push_back(posori_tasks[i]->_current_position);
		haptic_centers.push_back(Vector3d::Zero());

		teleop_tasks.push_back(new Sai2Primitives::HapticController(posori_tasks[i]->_current_position, posori_tasks[i]->_current_orientation, robot_pose_in_world[i].linear()));

		// Task scaling factors
		double Ks = 2.0;
		double KsR = 1.0;
		teleop_tasks[i]->setScalingFactors(Ks, KsR);

		// max stiffnesses
		Vector2d max_stiffness = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[i]);
		Vector2d max_damping = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[i]);
		Vector2d max_force = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[i]);

		teleop_tasks[i]->_max_linear_stiffness_device = max_stiffness(0);
		teleop_tasks[i]->_max_angular_stiffness_device = max_stiffness(1);
		teleop_tasks[i]->_max_linear_damping_device = max_damping(0);
		teleop_tasks[i]->_max_angular_damping_device = max_damping(1);
		teleop_tasks[i]->_max_force_device = max_force(0);
		teleop_tasks[i]->_max_torque_device = max_force(1);

	// _kp_position_ctrl_device = 0.15;
	// _kv_position_ctrl_device = 0.5;
	// _kp_orientation_ctrl_device = 0.5;
	// _kv_orientation_ctrl_device = 0.2;

		teleop_tasks[i]->setPosCtrlGains(0.15, 0.3, 0.5, 0.2);

		// gripper state
		gripper_state.push_back(false);
		previous_gripper_state.push_back(false);

		// passivity controller		
		passivity_controllers.push_back(new Sai2Primitives::BilateralPassivityController(posori_tasks[i], teleop_tasks[i]));
		passivity_damping_force.push_back(Vector3d::Zero());
		haptic_force_plus_passivity.push_back(Vector3d::Zero());

		// ee_inertial_forces
		ee_velocity.push_back(Vector3d::Zero());
		prev_ee_velocity.push_back(Vector3d::Zero());
		ee_acceleration.push_back(Vector3d::Zero());
		ee_inertial_forces.push_back(Vector3d::Zero());
	}

	// set force feedback for tasks
	teleop_tasks[0]->_send_haptic_feedback = true;
	teleop_tasks[0]->_haptic_feedback_from_proxy = false;
	teleop_tasks[0]->_filter_on = true;
	teleop_tasks[0]->setFilterCutOffFreq(0.01, 0.04);
	teleop_tasks[0]->setReductionFactorForceFeedback(0.6 * Matrix3d::Identity(), 1.0/20.0 * Matrix3d::Identity());

	teleop_tasks[1]->_send_haptic_feedback = true;
	teleop_tasks[1]->_haptic_feedback_from_proxy = false;
	teleop_tasks[1]->_filter_on = true;
	teleop_tasks[1]->setFilterCutOffFreq(0.01, 0.04);
	teleop_tasks[1]->setReductionFactorForceFeedback(0.6 * Matrix3d::Identity(), 1.0/20.0 * Matrix3d::Identity());

	// force sensor bias and end effector mass properties
	vector<VectorXd> force_sensor_bias;
	vector<VectorXd> bias_adjustment;
	vector<double> ee_mass;
	vector<Vector3d> ee_com_in_sensor_frame;

	for(int i=0 ; i<n_robots ; i++)
	{
		force_sensor_bias.push_back(VectorXd::Zero(6));
		bias_adjustment.push_back(VectorXd::Zero(6));
		ee_mass.push_back(0);
		ee_com_in_sensor_frame.push_back(Vector3d::Zero());
	}

	if(!flag_simulation)
	{
		force_sensor_bias[0] << -3.10306, 3.56132, 3.05367, 0.0680775, 0.306052, -0.0210922;
		ee_mass[0] =  1.33684;
		ee_com_in_sensor_frame[0] = Vector3d(0.0974865, 0.00583094,  0.0439404);

		force_sensor_bias[1] << -1.06547,    1.87883,   0.592603,  -0.113287,   0.394868, 0.00202288;
		ee_mass[1] =  0.275778;
		ee_com_in_sensor_frame[1] = Vector3d(0.0310245, -0.000648514,   0.00296009);
	}

	// setup redis keys to be updated with the callback
	// objects to read from redis
	vector<MatrixXd> mass_from_robots;
	vector<VectorXd> coriolis_from_robots;

	for(int i=0 ; i<n_robots ; i++)
	{
		mass_from_robots.push_back(MatrixXd::Identity(7,7));
		coriolis_from_robots.push_back(VectorXd::Zero(7));
	}

	for(int i=0 ; i<n_robots ; i++)
	{
		// read
		redis_client.addEigenToRead(JOINT_ANGLES_KEYS[i], robots[i]->_q);
		redis_client.addEigenToRead(JOINT_VELOCITIES_KEYS[i], robots[i]->_dq);
		
		redis_client.addEigenToRead(DEVICE_POSITION_KEYS[i], teleop_tasks[i]->_current_position_device);
		redis_client.addEigenToRead(DEVICE_ROTATION_KEYS[i], teleop_tasks[i]->_current_rotation_device);
		redis_client.addEigenToRead(DEVICE_TRANS_VELOCITY_KEYS[i], teleop_tasks[i]->_current_trans_velocity_device);
		redis_client.addEigenToRead(DEVICE_ROT_VELOCITY_KEYS[i], teleop_tasks[i]->_current_rot_velocity_device);
		redis_client.addEigenToRead(DEVICE_SENSED_FORCE_KEYS[i], teleop_tasks[i]->_sensed_force_device);
		redis_client.addEigenToRead(DEVICE_SENSED_TORQUE_KEYS[i], teleop_tasks[i]->_sensed_torque_device);
		redis_client.addDoubleToRead(DEVICE_GRIPPER_POSITION_KEYS[i], teleop_tasks[i]->_current_position_gripper_device);
		redis_client.addDoubleToRead(DEVICE_GRIPPER_VELOCITY_KEYS[i], teleop_tasks[i]->_current_gripper_velocity_device);

		if(!flag_simulation)
		{
			redis_client.addEigenToRead(MASSMATRIX_KEYS[i], mass_from_robots[i]);
			redis_client.addEigenToRead(CORIOLIS_KEYS[i], coriolis_from_robots[i]);
		}

		// write
		redis_client.addEigenToWrite(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
		redis_client.addEigenToWrite(DEVICE_COMMANDED_FORCE_KEYS[i], haptic_force_plus_passivity[i]);
		redis_client.addDoubleToWrite(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[i], teleop_tasks[i]->_commanded_gripper_force_device);
	}

	redis_client.addEigenToRead(FORCE_SENSED_KEYS[0], f_sensed[0]);
	redis_client.addEigenToRead(FORCE_SENSED_KEYS[1], f_sensed[1]);

	redis_client.addIntToRead(REMOTE_ENABLED_KEY, remote_enabled);
	redis_client.addIntToRead(RESTART_CYCLE_KEY, restart_cycle);

	redis_client.addEigenToWrite(ALLGERO_POSITION_COMMANDED_KEY, allegro_desired_config);
	redis_client.addEigenToWrite(ALLGERO_PALM_ORIENTATION_KEY, R_world_palm);

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

		// compute hand rotation for its gravity compensation
		robots[0]->rotation(R_world_ee, link_names[0]);
		R_world_palm = R_world_ee * R_ee_palm;
		
		// compute hand inertial forces
		for(int i=0 ; i<n_robots ; i++)
		{
			ee_velocity[i] = posori_tasks[i]->_current_velocity + posori_tasks[i]->_current_angular_velocity.cross(ee_com_in_sensor_frame[i]);
			if(controller_counter > 100)
			{
				ee_acceleration[i] = (ee_velocity[i] - prev_ee_velocity[i])/dt;
			}
			prev_ee_velocity[i] = ee_velocity[i];
			ee_inertial_forces[i] = ee_mass[i] * ee_acceleration[i];
		}


		// read force sensor data and remove bias and effects from hand gravity
		for(int i=0 ; i<n_robots ; i++)
		{
			f_sensed[i] -= force_sensor_bias[i] + bias_adjustment[i]; 
			Matrix3d R_sensor = Matrix3d::Identity();
			robots[i]->rotation(R_sensor, "link7");
			Vector3d p_tool_sensorFrame = ee_mass[i] * R_sensor.transpose() * Vector3d(0,0,-9.81); 
			f_sensed[i].head(3) += p_tool_sensorFrame;
			f_sensed[i].tail(3) += ee_com_in_sensor_frame[i].cross(p_tool_sensorFrame);

			f_sensed[i].head(3) -= 0.90 * R_sensor.transpose() * ee_inertial_forces[i];

			posori_tasks[i]->updateSensedForceAndMoment(f_sensed[i].head(3), f_sensed[i].tail(3));
			VectorXd sensed_force_world_frame = VectorXd::Zero(6);
			sensed_force_world_frame << posori_tasks[i]->_sensed_force, posori_tasks[i]->_sensed_moment;
			teleop_tasks[i]->updateSensedForce(-sensed_force_world_frame);
		}


		// use gripper as switch
		for(int i=0 ; i<n_robots ; i++)
		{
			teleop_tasks[i]->UseGripperAsSwitch();
			gripper_state[i] = teleop_tasks[i]->gripper_state;
		}

		// cout << gripper_state[0] << endl;

///////////////////////////////////////////////////////////////////////////////////////////
		//// State Machine ////
// 
		// 	same for both hands if state is goto init config or maintain position
		for(int i=0 ; i<n_robots ; i++)
		{
			if(state[i] == GOTO_INITIAL_CONFIG)
			{
				// update robot home position task model
				N_prec[i].setIdentity();
				joint_tasks[i]->updateTaskModel(N_prec[i]);
				// compute robot torques
				joint_tasks[i]->computeTorques(joint_task_torques[i]);
				command_torques[i] = joint_task_torques[i] + coriolis[i];
				
				// compute homing haptic device
				teleop_tasks[i]->HomingTask();

				if(remote_enabled==1 && (joint_tasks[i]->_desired_position - joint_tasks[i]->_current_position).norm() < 0.2 && teleop_tasks[i]->device_homed && gripper_state[i])
				{
					joint_tasks[i]->_ki = 0;
					posori_tasks[i]->reInitializeTask();
					workspace_centers[i] = posori_tasks[i]->_current_position;
					haptic_centers[i] = teleop_tasks[i]->_current_position_device;

					teleop_tasks[i]->setRobotCenter(workspace_centers[i], posori_tasks[i]->_current_orientation);
					teleop_tasks[i]->setDeviceCenter(haptic_centers[i], teleop_tasks[i]->_current_rotation_device);
					
					state[i] = HAPTIC_CONTROL;
				}
			}

			else if(state[i] == MAINTAIN_POSITION)
			{
				// update robot home position task model
				N_prec[i].setIdentity();
				joint_tasks[i]->updateTaskModel(N_prec[i]);
				// compute robot torques
				joint_tasks[i]->computeTorques(joint_task_torques[i]);
				command_torques[i] = joint_task_torques[i] + coriolis[i];

				// compute homing haptic device
				teleop_tasks[i]->HomingTask();

				if (remote_enabled==1 && gripper_state[i])
				{
					posori_tasks[i]->reInitializeTask();

					teleop_tasks[i]->setRobotCenter(workspace_centers[i], posori_tasks[i]->_current_orientation);
					teleop_tasks[i]->setDeviceCenter(haptic_centers[i], teleop_tasks[i]->_current_rotation_device);
					
					state[i] = HAPTIC_CONTROL;
				}
				else if (restart_cycle == 1)
				{
					// set joint controller to robot home position
					joint_tasks[i]->reInitializeTask();
					joint_tasks[i]->_desired_position = q_initial[i];
					// set haptic device home position
					teleop_tasks[i]->setDeviceCenter(haptic_centers[i], teleop_tasks[i]->_current_rotation_device);

					state[i] = GOTO_INITIAL_CONFIG;
				}
			}
		}

		if(state[0] == HAPTIC_CONTROL)
		{
			// update tasks model
			N_prec[0].setIdentity();
			posori_tasks[0]->updateTaskModel(N_prec[0]);
			N_prec[0] = posori_tasks[0]->_N;
			joint_tasks[0]->updateTaskModel(N_prec[0]);

			//compute haptic commands (only position control) - without force feedback (force_sensed=0)
			teleop_tasks[0]->computeHapticCommands3d(posori_tasks[0]->_desired_position);

			// compute robot set torques
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			command_torques[0] = joint_task_torques[0] + coriolis[0] + posori_task_torques[0];

			passivity_controllers[0]->computePOPCForce(passivity_damping_force[0]);

			if(gripper_state[0] && !previous_gripper_state[0])
			{
				// cout << "close gripper : " << close_gripper << endl;
				if(close_gripper)
				{
					allegro_desired_config = allegro_closed_config;
				}
				else
				{
					allegro_desired_config = allegro_open_config;
				}
			}
			else if(!gripper_state[0] && previous_gripper_state[0])
			{
				close_gripper = !close_gripper;
			}

			// if(!gripper_state[0])
			if(remote_enabled == 0)
			{
				// set joint controller to maintin robot in current position
				joint_tasks[0]->reInitializeTask();
				// joint_tasks[0]->_desired_position = robot[0]->_q;
				// set current haptic device position
				teleop_tasks[0]->setDeviceCenter(teleop_tasks[0]->_current_position_device, teleop_tasks[0]->_current_rotation_device);

				state[0] = MAINTAIN_POSITION;
			}			
		}
		else if(state[0] != GOTO_INITIAL_CONFIG && state[0] != MAINTAIN_POSITION)
		{
			command_torques[0].setZero(dof[0]);
			teleop_tasks[0]->GravityCompTask();			
		}

		if(state[1] == HAPTIC_CONTROL)
		{
			// update tasks model
			N_prec[1].setIdentity();
			posori_tasks[1]->updateTaskModel(N_prec[1]);
			N_prec[1] = posori_tasks[1]->_N;
			joint_tasks[1]->updateTaskModel(N_prec[1]);
			
			// compute haptic commands
			if(gripper_state[1]) //Full control
			{
				if(!previous_gripper_state[1])
				{
					teleop_tasks[1]->setDeviceCenter(haptic_centers[1], teleop_tasks[1]->_current_rotation_device);
					teleop_tasks[1]->setRobotCenter(workspace_centers[1], posori_tasks[1]->_current_orientation);
				
				}
				teleop_tasks[1]->computeHapticCommands6d( posori_tasks[1]->_desired_position,
																   posori_tasks[1]->_desired_orientation);

			}
			else //Only position control
			{
				teleop_tasks[1]->computeHapticCommands3d(posori_tasks[1]->_desired_position);
			}

			// compute robot set torques
			posori_tasks[1]->computeTorques(posori_task_torques[1]);
			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			command_torques[1] = joint_task_torques[1] + coriolis[1] + posori_task_torques[1];

			passivity_controllers[1]->computePOPCForce(passivity_damping_force[1]);

			if(remote_enabled == 0)
			{
				// set joint controller to maintin robot in current position
				joint_tasks[1]->reInitializeTask();
				// joint_tasks[1]->_desired_position = robot[1]->_q;
				// set current haptic device position
				teleop_tasks[1]->setDeviceCenter(teleop_tasks[1]->_current_position_device, teleop_tasks[1]->_current_rotation_device);

				state[1] = MAINTAIN_POSITION;
			}		
		}
		else if(state[1] != GOTO_INITIAL_CONFIG && state[1] != MAINTAIN_POSITION)
		{
			command_torques[1].setZero(dof[1]);
			teleop_tasks[1]->GravityCompTask();			
		}

		for(int i=0 ; i<n_robots ; i++)
		{
			previous_gripper_state[i] = gripper_state[i];
		}

		// send to redis
		for(int i=0 ; i<n_robots ; i++)
		{
			haptic_force_plus_passivity[i] = teleop_tasks[i]->_commanded_force_device + passivity_damping_force[i];
		}
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

		redis_client.set(ALLGERO_CONTROL_MODE_KEY, "t");
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
