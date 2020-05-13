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

const string robot_file = "./resources/panda_arm.urdf";

// redis keys:
// - read:
string JOINT_ANGLES_KEY = "sai2::PandaApplications::panda1::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::PandaApplications::panda1::sensors::dq";
string FORCE_SENSED_KEY = "sai2::PandaApplications::panda1::sensors::f_op";

const string REMOTE_ENABLED_KEY = "sai2::PandaApplications::sensors::remote_enabled";
const string RESTART_CYCLE_KEY = "sai2::PandaApplications::sensors::restart_cycle";

// - write
string JOINT_TORQUES_COMMANDED_KEY = "sai2::PandaApplications::panda1::actuators::fgc";

// - model
string MASSMATRIX_KEY = "sai2::PandaApplications::panda1::model::mass_matrix";
string CORIOLIS_KEY = "sai2::PandaApplications::panda1::model::coriolis";
string ROBOT_GRAVITY_KEY = "sai2::PandaApplications::panda1::model::gravity";

//// Haptic device related keys ////
// Maximum stiffness, damping and force specifications
const string DEVICE_MAX_STIFFNESS_KEY = "sai2::ChaiHapticDevice::device0::specifications::max_stiffness";
const string DEVICE_MAX_DAMPING_KEY = "sai2::ChaiHapticDevice::device0::specifications::max_damping";
const string DEVICE_MAX_FORCE_KEY = "sai2::ChaiHapticDevice::device0::specifications::max_force";
// Set force and torque feedback of the haptic device
const string DEVICE_COMMANDED_FORCE_KEY = "sai2::ChaiHapticDevice::device0::actuators::commanded_force";
const string DEVICE_COMMANDED_TORQUE_KEY = "sai2::ChaiHapticDevice::device0::actuators::commanded_torque";
const string DEVICE_COMMANDED_GRIPPER_FORCE_KEY = "sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper";
// Haptic device current position and rotation
const string DEVICE_POSITION_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_position";
const string DEVICE_ROTATION_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_rotation";
const string DEVICE_GRIPPER_POSITION_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_position_gripper";
// Haptic device current velocity
const string DEVICE_TRANS_VELOCITY_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity";
const string DEVICE_ROT_VELOCITY_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity";
const string DEVICE_GRIPPER_VELOCITY_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity";
const string DEVICE_SENSED_FORCE_KEY = "sai2::ChaiHapticDevice::device0::sensors::sensed_force";
const string DEVICE_SENSED_TORQUE_KEY = "sai2::ChaiHapticDevice::device0::sensors::sensed_torque";

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool inertia_regularization = true;

#define GOTO_INITIAL_CONFIG               0
#define HAPTIC_CONTROL                    1
#define MAINTAIN_POSITION				  2

int translation_counter = 0;

int remote_enabled = 1; //////////////////////////////////////////////////////// Read from redis ??
int restart_cycle = 0;

int state = GOTO_INITIAL_CONFIG;

unsigned long long controller_counter = 0;

RedisClient redis_client;

int main() {


	if(!flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";	
		FORCE_SENSED_KEY = "sai2::ATIGamma_Sensor::Bonnie::force_torque";
	}

	auto redis_client = RedisClient();
	redis_client.connect();

	redis_client.set(REMOTE_ENABLED_KEY, to_string(remote_enabled));
	redis_client.set(RESTART_CYCLE_KEY, to_string(restart_cycle));

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// position of robots in world
	Affine3d robot_pose_in_world = Affine3d::Identity();
	// robot_pose_in_world.translation() = Vector3d(-0.06, 0.57, 0.0);
	// robot_pose_in_world.linear() = AngleAxisd(-1.0864675, Vector3d::UnitZ()).toRotationMatrix();

	// load robot
	auto robot = new Sai2Model::Sai2Model(robot_file, false, robot_pose_in_world);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

	// prepare robot task controllers
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	VectorXd gravity = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	VectorXd q_initial = VectorXd::Zero(dof);
	// q_initial << -1.39968,-1.02909,1.04275,-1.84455,-0.357845,1.86703,-1.18647;
	q_initial = robot->_q;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	joint_task->_kp = 100.0;
	joint_task->_kv = 14.0;
	joint_task->_ki = 0.0;

	// joint_task->_desired_position = q_initial;
	joint_task->_otg->setMaxVelocity(M_PI/6);

	// end effector tasks
	const string link_name = "end_effector";
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.0);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	Affine3d sensor_frame = Affine3d::Identity();
	sensor_frame.translation() = Vector3d(0, 0, 0.12);
	posori_task->setForceSensorFrame(link_name, sensor_frame);

	posori_task->_use_interpolation_flag = false;
	posori_task->_use_velocity_saturation_flag = true;
	posori_task->_linear_saturation_velocity = 0.7;
	posori_task->_angular_saturation_velocity = M_PI/1.5;

	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 15.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 15.0;		

	// define home positions of the panda robot and haptic device
	Vector3d workspace_center_robot_trans = posori_task->_current_position;
	Vector3d workspace_center_haptic_trans = Vector3d::Zero();
	Matrix3d workspace_center_robot_rot = posori_task->_current_orientation;
	Matrix3d worlspace_center_haptic_rot = Matrix3d::Identity();

	// prepare haptic controller
	Matrix3d R_haptic_robot = robot_pose_in_world.linear();
	auto haptic_task = new Sai2Primitives::HapticController(workspace_center_robot_trans, workspace_center_robot_rot, R_haptic_robot);

	// haptic feedback
	haptic_task->_filter_on = true;
	haptic_task->setFilterCutOffFreq(0.004, 0.04);
	haptic_task->_haptic_feedback_from_proxy = false;
	haptic_task->_send_haptic_feedback = false;

	//Task scaling factors
	double haptic_scaling_translation = 2.0;
	double haptic_scaling_rotation = 1.0;
	haptic_task->setScalingFactors(haptic_scaling_translation, haptic_scaling_rotation);

	// virtual stiffness
	double kp_virtual = 200.0;
	double force_guidance_orientation_impedance = 7.0;
	double kv_virtual = 7.0;
	double force_guidance_orientation_damping = 0.02;
	haptic_task->setVirtualGuidanceGains (kp_virtual, kv_virtual,
									force_guidance_orientation_impedance, force_guidance_orientation_damping);

	Vector3d pos_rob_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_rob_model = Matrix3d::Identity();
	Vector3d vel_rot_rob_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_rob_model = Vector3d::Zero();

	// Add device workspace virtual limits 
	haptic_task->_add_workspace_virtual_limit = true;
	double device_workspace_radius_limit = 0.05;
	double device_workspace_angle_limit = 90*M_PI/180.0;
	haptic_task->setWorkspaceLimits(device_workspace_radius_limit, device_workspace_angle_limit);

	VectorXd sensed_force_robot_ee = VectorXd::Zero(6);
	VectorXd robot_sensor_force_bias = VectorXd::Zero(6);
	double robot_ee_mass = 0.0;
	Vector3d robot_ee_com = Vector3d::Zero();
	
	// if(!flag_simulation)
	// {
	// 	robot_sensor_force_bias << -0.133241,   0.413947,    15.3625, -0.0922695, 0.00362847, 0.00647149;
	// 	robot_ee_mass = 1.56353;
	// 	robot_ee_com = Vector3d(-6.19239e-05,  -0.00452189,    0.0777715);
	// }

	// Read haptic device specifications from haptic driver
	VectorXd _max_stiffness_haptic_device = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEY);
	VectorXd _max_damping_haptic_device = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEY);
	VectorXd _max_force_haptic_device = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEY);
	
	// set the device specifications to the haptic controller
	haptic_task->_max_linear_stiffness_device = _max_stiffness_haptic_device[0];
	haptic_task->_max_angular_stiffness_device = _max_stiffness_haptic_device[1];
	haptic_task->_max_linear_damping_device = _max_damping_haptic_device[0];
	haptic_task->_max_angular_damping_device = _max_damping_haptic_device[1];
	haptic_task->_max_force_device = _max_force_haptic_device[0];
	haptic_task->_max_torque_device = _max_force_haptic_device[1];

	bool haptic_gripper_state = false;
	bool previous_haptic_gripper_state = false;

	//// passivity observer and controller ////
	auto passivity_controller = new Sai2Primitives::BilateralPassivityController(posori_task, haptic_task);
	Vector3d haptic_damping_force_passivity = Vector3d::Zero();
	Vector3d command_force_device_plus_damping = Vector3d::Zero();

	// remove inertial forces from hand
	Vector3d robot_ee_velocity = Vector3d::Zero();
	Vector3d previous_robot_ee_velocity = Vector3d::Zero();
	Vector3d robot_ee_acceleration = Vector3d::Zero();
	Vector3d robot_ee_inertial_forces = Vector3d::Zero();

	// autonomous contact switching
	bool robot_in_contact = false;
	bool robot_in_contact_prev = false;
	Vector3d constraint_direction = Vector3d::UnitZ();


	// communication delay
	const int communication_delay = 5;
	queue<Vector3d> robot_position_buffer;
	queue<Vector3d> robot_velocity_buffer;
	queue<Vector3d> robot_desired_position_buffer;
	queue<Matrix3d> robot_orientation_buffer;
	queue<Vector3d> robot_angular_velocity_buffer;
	queue<Matrix3d> robot_desired_orientation_buffer;
	Vector3d current_robot_position;
	Vector3d delayed_robot_position;
	Vector3d current_robot_velocity;
	Vector3d delayed_robot_velocity;
	Vector3d current_robot_desired_position;
	Vector3d delayed_robot_desired_position;
	Matrix3d current_robot_orientation;
	Matrix3d delayed_robot_orientation;
	Matrix3d current_robot_desired_orientation;
	Matrix3d delayed_robot_desired_orientation;
	Vector3d current_robot_angular_velocity;
	Vector3d delayed_robot_angular_velocity;

	delayed_robot_position = posori_task->_current_position;
	delayed_robot_velocity.setZero();
	delayed_robot_orientation = posori_task->_current_orientation;
	delayed_robot_angular_velocity.setZero();
	delayed_robot_desired_position = posori_task->_current_position;
	delayed_robot_desired_orientation = posori_task->_current_orientation;

	// setup redis keys to be updated with the callback
	// objects to read from redis
	MatrixXd mass_from_robot_driver = MatrixXd::Identity(dof,dof);
	VectorXd coriolis_from_robot_driver = VectorXd::Zero(dof);

	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	redis_client.addEigenToReadCallback(0, FORCE_SENSED_KEY, sensed_force_robot_ee);

	redis_client.addEigenToReadCallback(0, DEVICE_POSITION_KEY, haptic_task->_current_position_device);
	redis_client.addEigenToReadCallback(0, DEVICE_ROTATION_KEY, haptic_task->_current_rotation_device);
	redis_client.addEigenToReadCallback(0, DEVICE_TRANS_VELOCITY_KEY, haptic_task->_current_trans_velocity_device);
	redis_client.addEigenToReadCallback(0, DEVICE_ROT_VELOCITY_KEY, haptic_task->_current_rot_velocity_device);
	redis_client.addEigenToReadCallback(0, DEVICE_SENSED_FORCE_KEY, haptic_task->_sensed_force_device);
	redis_client.addEigenToReadCallback(0, DEVICE_SENSED_TORQUE_KEY, haptic_task->_sensed_torque_device);
	redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_POSITION_KEY, haptic_task->_current_position_gripper_device);
	redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_VELOCITY_KEY, haptic_task->_current_gripper_velocity_device);

	redis_client.addIntToReadCallback(0, REMOTE_ENABLED_KEY, remote_enabled);
	redis_client.addIntToReadCallback(0, RESTART_CYCLE_KEY, restart_cycle);

	if(!flag_simulation)
	{
		redis_client.addEigenToReadCallback(0, MASSMATRIX_KEY, mass_from_robot_driver);
		redis_client.addEigenToReadCallback(0, CORIOLIS_KEY, coriolis_from_robot_driver);
	}

	// objects to write to redis
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_FORCE_KEY, command_force_device_plus_damping);
	redis_client.addDoubleToWriteCallback(0, DEVICE_COMMANDED_GRIPPER_FORCE_KEY, haptic_task->_commanded_gripper_force_device);


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

		if(controller_counter % 100 == 0)
		{
			// cout << "device position :\n" << haptic_task->_current_position_device.transpose() << endl;
			// cout << "device rotation :\n" << haptic_task->_current_rotation_device.transpose() << endl;
			// cout << "robot position :\n" << posori_task->_current_position.transpose() << endl;
			// cout << "commanded force haptic device :\n" << haptic_task->_commanded_force_device.transpose() << endl;
			// cout << endl;
		}

		// update model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);
		}
		else
		{
			robot->updateKinematics();
			robot->_M = mass_from_robot_driver;
			robot->updateInverseInertia();
			coriolis = coriolis_from_robot_driver;
		}

		// Update position and orientation of the robot from the model
		robot->position(current_robot_position, link_name, pos_in_link);
		robot->linearVelocity(current_robot_velocity, link_name, pos_in_link);
		robot->rotation(current_robot_orientation, link_name);
		robot->angularVelocity(current_robot_angular_velocity, link_name);

		robot_position_buffer.push(current_robot_position);
		robot_velocity_buffer.push(current_robot_velocity);
		robot_orientation_buffer.push(current_robot_orientation);
		robot_angular_velocity_buffer.push(current_robot_angular_velocity);
		if(robot_position_buffer.size() > communication_delay)
		{
			delayed_robot_position = robot_position_buffer.front();
			delayed_robot_velocity = robot_velocity_buffer.front();
			delayed_robot_orientation = robot_orientation_buffer.front();
			delayed_robot_angular_velocity = robot_angular_velocity_buffer.front();

			robot_position_buffer.pop();
			robot_velocity_buffer.pop();
			robot_orientation_buffer.pop();
			robot_angular_velocity_buffer.pop();
		}

		haptic_task->updateSensedRobotPositionVelocity(delayed_robot_position, delayed_robot_velocity,
														delayed_robot_orientation, delayed_robot_angular_velocity);

		// compute hand inertial forces
		robot->linearVelocity(robot_ee_velocity, link_name, robot_ee_com);

		// acceleration from velocity differentiation
		if(controller_counter > 100)
		{
			robot_ee_acceleration = (robot_ee_velocity - previous_robot_ee_velocity)/dt;
		}
		previous_robot_ee_velocity = robot_ee_velocity;

		// acceleration from model (if joint accelerations updated)
		// robot->linearAcceleration(robot_ee_acceleration, link_name, robot_ee_com);

		// inertial forces
		robot_ee_inertial_forces = robot_ee_mass * robot_ee_acceleration;

		// read force sensor data and remove bias and effecto from hand gravity
		robot_in_contact_prev = robot_in_contact;
		if(controller_counter % 100 == 0)
		{
			if(sensed_force_robot_ee.norm() > 1)
			{
				robot_in_contact = true;
			}
			else
			{
				robot_in_contact = false;
			}
		}

		sensed_force_robot_ee -= robot_sensor_force_bias; 
		Matrix3d R_robot_sensor = Matrix3d::Identity();
		robot->rotation(R_robot_sensor, link_name, sensor_frame.linear());
		Vector3d p_tool_sensorFrame_ee = robot_ee_mass * R_robot_sensor.transpose() * Vector3d(0,0,-9.81); 
		sensed_force_robot_ee.head(3) += p_tool_sensorFrame_ee;
		sensed_force_robot_ee.tail(3) += robot_ee_com.cross(p_tool_sensorFrame_ee);

		sensed_force_robot_ee.head(3) -= 0.8 * R_robot_sensor.transpose() * robot_ee_inertial_forces;

		posori_task->updateSensedForceAndMoment(sensed_force_robot_ee.head(3), sensed_force_robot_ee.tail(3));
		VectorXd sensed_force_robot_ee_world_frame = VectorXd::Zero(6);
		sensed_force_robot_ee_world_frame << posori_task->_sensed_force, posori_task->_sensed_moment;
		
		// update haptic task
		haptic_task->updateSensedForce(-sensed_force_robot_ee_world_frame);
		haptic_task->UseGripperAsSwitch();

///////////////////////////////////////////////////////////////////////////////////////////
		if(state == GOTO_INITIAL_CONFIG)
		{
			// update robot home position task model
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			// compute robot torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques + coriolis;
			
			// compute homing haptic device
			haptic_task->HomingTask();

			// read gripper state
			haptic_gripper_state = haptic_task->gripper_state;

			if(remote_enabled==1 && (joint_task->_desired_position - joint_task->_current_position).norm() < 0.2 && haptic_task->device_homed && haptic_gripper_state)
			{
				joint_task->_ki = 0;
				posori_task->reInitializeTask();
				workspace_center_robot_trans = posori_task->_current_position;
				workspace_center_haptic_trans = haptic_task->_current_position_device;

				haptic_task->reInitializeTask();
				haptic_task->setRobotCenter(workspace_center_robot_trans, posori_task->_current_orientation);
				haptic_task->setDeviceCenter(workspace_center_haptic_trans, haptic_task->_current_rotation_device);
				
				haptic_task->_haptic_feedback_from_proxy = false; // If set to true, the force feedback is computed from a stiffness/damping proxy.
			    haptic_task->_filter_on = true;
				haptic_task->_send_haptic_feedback = true;

				state = HAPTIC_CONTROL;
			}
		}

		else if(state == HAPTIC_CONTROL)
		{

			// update tasks model
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);
			
			if(robot_in_contact && !robot_in_contact_prev)
			{
				posori_task->setForceAxis(constraint_direction);
			}
			else if(robot_in_contact)
			{
				posori_task->updateForceAxis(constraint_direction);
			}
			else
			{
				posori_task->setFullLinearMotionControl();
			}

			haptic_task->updateSelectionMatrices(Matrix3d::Zero(), Matrix3d::Zero(),
							 						posori_task->_sigma_force, Matrix3d::Zero());

			// // read gripper state
			haptic_gripper_state = haptic_task->gripper_state;
			// compute haptic commands
			if(haptic_gripper_state) //Full control
			{
				if(!previous_haptic_gripper_state)
				{
					haptic_task->setDeviceCenter(workspace_center_haptic_trans, haptic_task->_current_rotation_device);
					haptic_task->setRobotCenter(workspace_center_robot_trans, posori_task->_current_orientation);
				
				}
				// Matrix3d desired_rotation_relative = Matrix3d::Identity();
				Vector3d moment_dummy = Vector3d::Zero();
				Vector3d force_dummy = Vector3d::Zero();
				haptic_task->computeHapticCommandsUnifiedControl6d( current_robot_desired_position,
																   current_robot_desired_orientation,
																   force_dummy,
																   moment_dummy);

			}
			else //Only position control
			{
				Vector3d force_dummy = Vector3d::Zero();
				haptic_task->computeHapticCommandsUnifiedControl3d(current_robot_desired_position,
																	force_dummy);
			}


			robot_desired_position_buffer.push(current_robot_desired_position);
			robot_desired_orientation_buffer.push(current_robot_desired_orientation);

			if(robot_desired_position_buffer.size() > communication_delay)
			{
				delayed_robot_desired_position = robot_desired_position_buffer.front();
				delayed_robot_desired_orientation = robot_desired_orientation_buffer.front();

				robot_desired_position_buffer.pop();
				robot_desired_orientation_buffer.pop();
			}

			posori_task->_desired_force = - kp_virtual * (current_robot_position - delayed_robot_desired_position) - kv_virtual * current_robot_velocity;
			posori_task->_desired_position = delayed_robot_desired_position;
			posori_task->_desired_orientation = delayed_robot_desired_orientation;
			// posori_task->_desired_force(2) -= 2.0;

			// compute robot set torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis + posori_task_torques;

			// passivity_controller->computePOPCForce(haptic_damping_force_passivity);

			if(remote_enabled == 0)
			{
				// set joint controller to maintin robot in current position
				joint_task->reInitializeTask();
				// set current haptic device position
				haptic_task->setDeviceCenter(haptic_task->_current_position_device, haptic_task->_current_rotation_device);

				state = MAINTAIN_POSITION;
			}
		}

		else if(state == MAINTAIN_POSITION)
		{
			// update robot home position task model
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			// compute robot torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques + coriolis;

			// compute homing haptic device
			haptic_task->HomingTask();

			// read gripper state
			haptic_gripper_state = haptic_task->gripper_state;



			if (remote_enabled==1 && haptic_gripper_state)
			{
				posori_task->reInitializeTask();

				haptic_task->setRobotCenter(workspace_center_robot_trans, posori_task->_current_orientation);
				haptic_task->setDeviceCenter(workspace_center_haptic_trans, haptic_task->_current_rotation_device);
				
				state = HAPTIC_CONTROL;
			}
			else if (restart_cycle == 1)
			{
				// set joint controller to robot home position
				joint_task->reInitializeTask();
				joint_task->_desired_position = q_initial;
				// set haptic device home position
				haptic_task->setDeviceCenter(workspace_center_haptic_trans, haptic_task->_current_rotation_device);

				state = GOTO_INITIAL_CONFIG;
			}
		}
		else
		{
			command_torques.setZero(dof);
			haptic_task->GravityCompTask();
		}
		previous_haptic_gripper_state = haptic_task->gripper_state;

		// send to redis
		command_force_device_plus_damping = haptic_task->_commanded_force_device + haptic_damping_force_passivity;
		// if(state == HAPTIC_CONTROL)
		// {
		// 	command_force_device_plus_damping.setZero();
		// }
		redis_client.executeWriteCallback(0);

		prev_time = current_time;
		controller_counter++;
	}

	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEY, Vector3d::Zero());
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEY, Vector3d::Zero());
	redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEY, "0.0");

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
