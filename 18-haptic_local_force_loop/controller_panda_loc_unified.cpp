// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "haptic_tasks/HapticController.h"
#include "filters/ButterworthFilter.h"
#include "Logger.h"

#include "ForceSpaceParticleFilter_weight_mem.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>
#include <random>
#include <queue>

#define INIT            0
#define CONTROL         1

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string link_name = "end_effector"; //robot end-effector
// Set sensor frame transform in end-effector frame
Affine3d sensor_transform_in_link = Affine3d::Identity();
const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);
const Vector3d pos_in_link = Vector3d(0.0,0.0,0.12);

// redis keys:
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

string JOINT_ANGLES_KEY = "sai2::PandaApplications::18::simviz_panda::sensors::sphere_pos";
string JOINT_VELOCITIES_KEY = "sai2::PandaApplications::18::simviz_panda::sensors::sphere_vel";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::PandaApplications::18::simviz_panda::actuators::command_torques";

string ROBOT_SENSED_FORCE_KEY = "sai2::PandaApplications::18::simviz_panda::sensors::sensed_force";

string PARTICLE_POSITIONS_KEY = "sai2::PandaApplications::18::simviz_panda::particle_positions";

string MASSMATRIX_KEY;
string CORIOLIS_KEY;

string DEBUG_SENSED_FORCE_WORLD_FRAME = "sai2::PandaApplications::18::debug::sensed_force_world_frame";

RedisClient redis_client;

// simulation function prototype
void particle_filter();
void communication();

Vector3d delayed_robot_position = Vector3d::Zero();
Vector3d delayed_haptic_position = Vector3d::Zero();
Vector3d delayed_haptic_velocity = Vector3d::Zero();
Vector3d delayed_sensed_force = Vector3d::Zero();
Matrix3d delayed_sigma_force = Matrix3d::Zero();

Vector3d robot_position_global = Vector3d::Zero();
Vector3d haptic_position_global = Vector3d::Zero();
Vector3d haptic_velocity_global = Vector3d::Zero();
Vector3d sensed_force_global = Vector3d::Zero();
Matrix3d sigma_force_global = Matrix3d::Zero();
Matrix3d sigma_motion_global = Matrix3d::Identity();

Vector3d motion_control_pfilter;
Vector3d force_control_pfilter;
Vector3d measured_velocity_pfilter;
Vector3d measured_force_pfilter;

// particle filter parameters
const int n_particles = 1000;
MatrixXd particle_positions_to_redis;

// const double percent_chance_contact_appears = 0.01;
const double percent_chance_contact_disapears = 0.95;
const double mean_scatter = 0.0;
const double std_scatter = 0.01;

const double coeff_friction = 0.0;

int force_space_dimension = 0;
int previous_force_space_dimension = 0;
Vector3d force_axis = Vector3d::Zero();
Vector3d motion_axis = Vector3d::Zero();

bool adding_contact = false;
bool removing_contact = false;
const int contact_transition_steps = 200;
int contact_transition_current_step = 0;

Matrix3d sigma_force;

// logger
Vector3d log_robot_position = Vector3d::Zero();
Vector3d log_robot_velocity = Vector3d::Zero();
Vector3d log_haptic_position = Vector3d::Zero();
Vector3d log_haptic_velocity = Vector3d::Zero();
Vector3d log_robot_force = Vector3d::Zero();
Vector3d log_haptic_force = Vector3d::Zero();
Vector3d log_sensed_force = Vector3d::Zero();
Vector3d log_eigenvalues = Vector3d::Zero();
Vector3d log_eigenvector_0 = Vector3d::Zero();
Vector3d log_eigenvector_1 = Vector3d::Zero();
Vector3d log_eigenvector_2 = Vector3d::Zero();

Vector3d log_force_axis = Vector3d::Zero();
Vector3d log_motion_axis = Vector3d::Zero();

VectorXd log_force_space_dimension = VectorXd::Zero(1);


// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

const bool flag_simulation = false;
// const bool flag_simulation = true;

int main() {

	if(!flag_simulation)
	{
		ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::Bonnie::force_torque";
	}

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// prepare particles
	particle_positions_to_redis = MatrixXd::Zero(3,n_particles);

	// load robots
	Affine3d T_workd_robot = Affine3d::Identity();
	T_workd_robot.translation() = Vector3d(0, 0, 0);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_workd_robot);

	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	int state = INIT;
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = false;

	// VectorXd q_init(robot->dof());
	// q_init << 0, 30, 0, -90, 0, 120, 0;
	// q_init *= M_PI/180.0;

	// joint_task->_desired_position = q_init;

	// posori task
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	Vector3d x_init = posori_task->_current_position;
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = false;
	posori_task->_linear_saturation_velocity = 0.35;

	posori_task->_otg->setMaxLinearVelocity(0.20);
	posori_task->_otg->setMaxLinearAcceleration(10.0);
	posori_task->_otg->setMaxLinearJerk(50.0);

	Vector3d proxy_error_robot_pos = Vector3d::Zero();
	Vector3d prev_proxy_error_robot_pos = Vector3d::Zero();

	haptic_position_global = posori_task->_current_position;

	posori_task->_kp_pos = 500.0;
	posori_task->_kv_pos = 20.0;

	// posori_task->_kp_ori = 10.0;
	posori_task->_kp_ori = 600.0;
	posori_task->_kv_ori = 38.0;

	sensor_transform_in_link.translation() = sensor_pos_in_link;
	posori_task->setForceSensorFrame(link_name, sensor_transform_in_link);

	// posori_task->setOpenLoopForceControl();
	posori_task->setClosedLoopForceControl();
	posori_task->_passivity_enabled = false;

	posori_task->_kp_force = 0.5;
	posori_task->_ki_force = 1.7;
	posori_task->_kv_force = 25.0;

	double k_vir_robot = 500.0;
	const double k_vir_haptic_goal = 500.0;
	double k_vir_haptic = k_vir_haptic_goal;
	// Matrix3d k_vir_haptic = k_vir_haptic_goal * Matrix3d::Ones();
	Vector3d robot_proxy_diff = Vector3d::Zero();
	Vector3d haptic_proxy_diff = Vector3d::Zero();

	double haptic_PO = 0;

	const double max_force_diff_robot = 0.05;
	const double max_force_diff_haptic = 0.05;
	Vector3d prev_desired_force_robot = Vector3d::Zero();
	Vector3d prev_desired_force_haptic = Vector3d::Zero();

	auto filter_force_command_robot = new ButterworthFilter(3,0.01);
	auto filter_force_command_haptic = new ButterworthFilter(3,0.01);

	// auto filter_sensed_force = new ButterworthFilter(6,0.1);

	// auto filter_sensed_force = new ButterworthFilter(3, 0.45);
	// Vector3d filtered_sensed_force = Vector3d::Zero();

	double kp_force = 0.0;
	double ki_force = 0.0;
	Vector3d integrated_force_error = Vector3d::Zero();

	VectorXd sensed_force_moment_local_frame_raw = VectorXd::Zero(6);
	VectorXd sensed_force_moment_local_frame = VectorXd::Zero(6);
	VectorXd sensed_force_moment_world_frame = VectorXd::Zero(6);
	VectorXd force_bias = VectorXd::Zero(6);
	double tool_mass = 0;
	Vector3d tool_com = Vector3d::Zero();

	Vector3d init_force = Vector3d::Zero();
	bool first_loop = true;

	if(!flag_simulation)
	{
		// force_bias << -0.434879,    1.99348,  -0.195705, -0.0886675,   0.394051,  0.0285245;
		// force_bias << -0.39392,    1.9322, -0.455304, -0.085451,  0.396084, 0.0322822;
		force_bias << -0.407025,    2.03665, -0.0510554, -0.0856087,   0.393342,  0.0273708;
		tool_mass = 0.361898;
		tool_com = Vector3d(-4.76184e-05, -0.000655773,    0.0354622);
	}

	// remove inertial forces from tool
	Vector3d tool_velocity = Vector3d::Zero();
	Vector3d prev_tool_velocity = Vector3d::Zero();
	Vector3d tool_acceleration = Vector3d::Zero();
	Vector3d tool_inertial_forces = Vector3d::Zero();


	// haptic task
	////Haptic teleoperation controller ////
	auto teleop_task = new Sai2Primitives::HapticController(x_init, Matrix3d::Zero());
	teleop_task->_send_haptic_feedback = false;

	//User switch states
	teleop_task->UseGripperAsSwitch();
	bool gripper_state = false;
	bool gripper_state_prev = false;

	 //Task scaling factors
	double Ks = 2.5;
	double KsR = 1.0;
	teleop_task->setScalingFactors(Ks, KsR);

	double kv_haptic = 22.0;

	int contact_transition_counter = 50;

	// particle filter buffers
	double freq_ratio_filter_control = 15.0 / 1000.0;
	queue<Vector3d> pfilter_motion_control_buffer;
	queue<Vector3d> pfilter_force_control_buffer;
	queue<Vector3d> pfilter_sensed_force_buffer;
	queue<Vector3d> pfilter_sensed_velocity_buffer;

	// // Center of the haptic device workspace
	// Vector3d HomePos_op;
	// HomePos_op << 0.0, 0.0, 0.0;
	// Matrix3d HomeRot_op;
	// HomeRot_op.setIdentity();
	// teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

	// double force_guidance_position_impedance = 1000.0;
	// double force_guidance_orientation_impedance = 50.0;
	// double force_guidance_position_damping = 5.0;
	// double force_guidance_orientation_damping = 0.1;
	// teleop_task->setVirtualGuidanceGains (force_guidance_position_impedance, force_guidance_position_damping,
	// 								force_guidance_orientation_impedance, force_guidance_orientation_damping);

	VectorXd _max_stiffness_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0]);
	VectorXd _max_damping_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0]);
	VectorXd _max_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0]);

	//set the device specifications to the haptic controller
	teleop_task->_max_linear_stiffness_device = _max_stiffness_device0[0];
	teleop_task->_max_angular_stiffness_device = _max_stiffness_device0[1];
	teleop_task->_max_linear_damping_device = _max_damping_device0[0];
	teleop_task->_max_angular_damping_device = _max_damping_device0[1];
	teleop_task->_max_force_device = _max_force_device0[0];
	teleop_task->_max_torque_device = _max_force_device0[1];

	// setup redis keys to be updated with the callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// Objects to read from redis
    redis_client.addEigenToReadCallback(0, DEVICE_POSITION_KEYS[0], teleop_task->_current_position_device);
    redis_client.addEigenToReadCallback(0, DEVICE_ROTATION_KEYS[0], teleop_task->_current_rotation_device);
    redis_client.addEigenToReadCallback(0, DEVICE_TRANS_VELOCITY_KEYS[0], teleop_task->_current_trans_velocity_device);
    redis_client.addEigenToReadCallback(0, DEVICE_ROT_VELOCITY_KEYS[0], teleop_task->_current_rot_velocity_device);
    redis_client.addEigenToReadCallback(0, DEVICE_SENSED_FORCE_KEYS[0], teleop_task->_sensed_force_device);
    redis_client.addEigenToReadCallback(0, DEVICE_SENSED_TORQUE_KEYS[0], teleop_task->_sensed_torque_device);
    redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_POSITION_KEYS[0], teleop_task->_current_position_gripper_device);
    redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_VELOCITY_KEYS[0], teleop_task->_current_gripper_velocity_device);

    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

    redis_client.addEigenToReadCallback(0, ROBOT_SENSED_FORCE_KEY, sensed_force_moment_local_frame_raw);

    MatrixXd mass_from_robot = MatrixXd::Identity(dof,dof);
    VectorXd coriolis_from_robot = VectorXd::Zero(dof);
	if(!flag_simulation)
	{
		redis_client.addEigenToReadCallback(0, MASSMATRIX_KEY, mass_from_robot);
		redis_client.addEigenToReadCallback(0, CORIOLIS_KEY, coriolis_from_robot);
	}

	// Objects to write to redis
	//write haptic commands
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_FORCE_KEYS[0], teleop_task->_commanded_force_device);
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_TORQUE_KEYS[0], teleop_task->_commanded_torque_device);
	redis_client.addDoubleToWriteCallback(0, DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], teleop_task->_commanded_gripper_force_device);

	redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

	Vector3d debug_force_world_frame = Vector3d::Zero();
	redis_client.addEigenToWriteCallback(0, DEBUG_SENSED_FORCE_WORLD_FRAME, debug_force_world_frame);

	// logger
	string folder = "../../18-haptic_local_force_loop/data_logging/data/";
	string timestamp = currentDateTime();
	string prefix = "data";
	string suffix = ".csv";
	string filename = folder + prefix + "_" + timestamp + suffix;
	auto logger = new Logging::Logger(1000, filename);
	
	logger->addVectorToLog(&log_robot_position, "robot_position");
	logger->addVectorToLog(&log_robot_velocity, "robot_velocity");
	logger->addVectorToLog(&log_haptic_position, "haptic_position");
	logger->addVectorToLog(&log_haptic_velocity, "haptic_velocity");
	logger->addVectorToLog(&log_robot_force, "robot_force");
	logger->addVectorToLog(&log_haptic_force, "haptic_force");
	logger->addVectorToLog(&log_sensed_force, "sensed_force");
	logger->addVectorToLog(&log_eigenvalues, "eigenvalues");
	logger->addVectorToLog(&log_eigenvector_0, "eigenvector_0");
	logger->addVectorToLog(&log_eigenvector_1, "eigenvector_1");
	logger->addVectorToLog(&log_eigenvector_2, "eigenvector_2");

	logger->addVectorToLog(&log_force_axis, "force_axis");
	logger->addVectorToLog(&log_motion_axis, "motion_axis");

	logger->addVectorToLog(&log_force_space_dimension, "force_space_dimension");

	logger->start();

	runloop = true;
	thread particle_filter_thread(particle_filter);
	thread communication_thread(communication);

	// create a timer
	double control_loop_freq = 1000.0;
	unsigned long long controller_counter = 0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(control_loop_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while (runloop)
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;

		// read haptic state and robot state
		redis_client.executeReadCallback(0);
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);
		}
		else
		{
			robot->updateKinematics();
			robot->_M = mass_from_robot;
			robot->updateInverseInertia();
			coriolis = coriolis_from_robot;
		}

		// compute tool inertial forces

		if(controller_counter > 100)
		{
			tool_acceleration = (tool_velocity - prev_tool_velocity) * control_loop_freq;
		}
		prev_tool_velocity = tool_velocity;
		tool_inertial_forces = tool_mass * tool_acceleration;


		sensed_force_moment_local_frame = sensed_force_moment_local_frame_raw;
		// sensed_force_moment_local_frame = filter_sensed_force->update(sensed_force_moment_local_frame_raw);

		// add bias and ee weight to sensed forces
		sensed_force_moment_local_frame -= force_bias;
		Matrix3d R_world_sensor;
		robot->rotation(R_world_sensor, link_name);
		Vector3d p_tool_local_frame = tool_mass * R_world_sensor.transpose() * Vector3d(0,0,-9.81);
		sensed_force_moment_local_frame.head(3) += p_tool_local_frame;
		sensed_force_moment_local_frame.tail(3) += tool_com.cross(p_tool_local_frame);

		if(first_loop)
		{
			init_force = sensed_force_moment_local_frame.head(3);
			first_loop = false;
		}
		sensed_force_moment_local_frame.head(3) -= init_force;

		// sensed_force_moment_local_frame.head(3) -= 0.7 * R_world_sensor.transpose() * tool_inertial_forces;

		// filter sensed force
		// filtered_sensed_force = filter_sensed_force->update(sensed_force_moment_local_frame.head(3));
		// sensed_force_moment_local_frame.head(3) = filtered_sensed_force;

		// update forces for posori task
		posori_task->updateSensedForceAndMoment(sensed_force_moment_local_frame.head(3), sensed_force_moment_local_frame.tail(3));

		sensed_force_moment_world_frame.head(3) = R_world_sensor * sensed_force_moment_local_frame.head(3);
		sensed_force_moment_world_frame.tail(3) = R_world_sensor * sensed_force_moment_local_frame.tail(3);


		debug_force_world_frame = sensed_force_moment_world_frame.head(3);



		// cout << "sensed force moment local frame after ee compensation : " << sensed_force_moment_local_frame.transpose() << endl;
		// cout << "sensed force moment world frame : " << sensed_force_moment_world_frame.transpose() << endl;
		// cout << "sensed force world frame in task : " << posori_task->_sensed_force.transpose() << endl;
		// cout << endl;

		N_prec.setIdentity(dof,dof);

		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;

		// if(!flag_simulation)
		// {
		// 	double coupling_correction = 1.0;
		// 	posori_task->_Lambda_modified(0,2) += coupling_correction;
		// 	posori_task->_Lambda_modified(2,0) += coupling_correction;
		// }

		// 
		// posori_task->_Lambda_modified(0,2) = 0;
		// posori_task->_Lambda_modified(2,0) = 0;

		joint_task->updateTaskModel(N_prec);

		teleop_task->UseGripperAsSwitch();
		gripper_state_prev = gripper_state;
		gripper_state = teleop_task->gripper_state;


		if(state == INIT)
		{
			joint_task->updateTaskModel(MatrixXd::Identity(dof,dof));

			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

  			// compute homing haptic device
  			teleop_task->HomingTask();

			// if((posori_task->_desired_position - posori_task->_current_position).norm() < 0.2)
			if(teleop_task->device_homed && gripper_state && (joint_task->_desired_position - joint_task->_current_position).norm() < 0.2)
			{
				teleop_task->setRobotCenter(posori_task->_current_position);
				teleop_task->setDeviceCenter(teleop_task->_current_position_device);

				// Reinitialize controllers
				posori_task->reInitializeTask();
				joint_task->reInitializeTask();
				teleop_task->reInitializeTask();

				posori_task->_desired_position = posori_task->_current_position;
				// delayed_haptic_position = posori_task->_current_position;


				// posori_task->_kp_pos = 200.0;
				// posori_task->_kv_pos = 30.0;

				state = CONTROL;
			}
		}

		else if(state == CONTROL)
		{

			// posori_task->setForceAxis(Vector3d::UnitZ());
			// sigma_force_global = Matrix3d::Zero();
			// sigma_force_global(2,2) = 1;

			if(force_space_dimension == 1)
			{
				if(previous_force_space_dimension == 1)
				{
					posori_task->updateForceAxis(force_axis);
				}
				else
				{
					posori_task->setForceAxis(force_axis);
				}
				sigma_force_global = force_axis * force_axis.transpose();
			}
			else if(force_space_dimension == 2)
			{
				if(previous_force_space_dimension == 2)
				{
					posori_task->updateLinearMotionAxis(motion_axis);
				}
				else
				{
					posori_task->setLinearMotionAxis(motion_axis);
				}
				sigma_force_global = Matrix3d::Identity() - motion_axis * motion_axis.transpose();
			}
			else if(force_space_dimension == 3)
			{
				if(previous_force_space_dimension != 3)
				{
					posori_task->setFullForceControl();
				}
				sigma_force_global.setIdentity();
			}
			else
			{
				if(previous_force_space_dimension != 0)
				{
					posori_task->setFullLinearMotionControl();
				}
				sigma_force_global.setZero();
			}


			// cout << "Lambda :\n" << posori_task->_Lambda << endl;
			// cout << "Lambda modified :\n" << posori_task->_Lambda_modified << endl;
			// cout << endl;

			Vector3d desired_position = Vector3d::Zero();
			teleop_task->computeHapticCommands3d(desired_position);
			haptic_position_global = desired_position;

			haptic_velocity_global = teleop_task->_current_trans_velocity_device_RobFrame;
			// Vector3d desired_velocity = teleop_task->_current_trans_velocity_device_RobFrame;
			// desired_position = teleop_task->_current_position_device;

			// cout << "haptic pos in robot frame : " << desired_position.transpose() << endl;
			// cout << "delayed haptic pos in robot frame : " << delayed_haptic_position.transpose() << endl;
			// cout << "robot pos : " << posori_task->_current_position.transpose() << endl;
			// cout << endl;


			// double interpolation_step = 0.1;

			// Vector3d new_proxy_error_robot_pos = (Matrix3d::Identity() - sigma_force_global) * (posori_task->_current_position - delayed_haptic_position);

			// Vector3d proxy_error_increment = new_proxy_error_robot_pos - prev_proxy_error_robot_pos;
			// if(proxy_error_increment.norm() > interpolation_step)
			// {
			// 	proxy_error_robot_pos = prev_proxy_error_robot_pos + interpolation_step * proxy_error_increment/proxy_error_increment.norm(); 
			// }
			// else
			// {
			// 	proxy_error_robot_pos = new_proxy_error_robot_pos;
			// }


			// posori_task->_desired_position = posori_task->_current_position + proxy_error_robot_pos;

			// cout << "prev proxy error robot pos : " << prev_proxy_error_robot_pos.transpose() << endl;
			// cout << "proxy error robot pos : " << proxy_error_robot_pos.transpose() << endl;

			// posori_task->_desired_position = delayed_haptic_position;
			posori_task->_desired_position = posori_task->_current_position + (Matrix3d::Identity() - sigma_force_global) * (delayed_haptic_position - posori_task->_current_position);

			// cout << posori_task->_desired_position.transpose() << endl;
			// cout << posori_task->_current_position.transpose() << endl;
			// cout << endl;
			// posori_task->_desired_velocity = delayed_haptic_velocity;

			robot_proxy_diff = posori_task->_current_position - delayed_haptic_position;
			Vector3d desired_force_robot = - k_vir_robot * sigma_force_global * robot_proxy_diff;

			Vector3d projected_desired_force = sigma_force_global * robot_proxy_diff;
			if(projected_desired_force.norm() > 1e-4)
			{
				desired_force_robot -= 0.0 * projected_desired_force / projected_desired_force.norm();
			}

			// cout << "desired force robot : " << desired_force_robot.transpose() << endl;

			// Vector3d desired_force_robot = posori_task->_sigma_force * k_vir_robot * robot_proxy_diff;
			Vector3d force_diff_robot = desired_force_robot - prev_desired_force_robot;
			if( force_diff_robot.norm() > max_force_diff_robot )
			{
			// 	// cout << "prev force :\n" << prev_desired_force_robot.transpose() << endl;
			// 	// cout << "new force command :\n" << force_command.transpose() << endl;
			// 	// cout << endl;
				desired_force_robot = prev_desired_force_robot + max_force_diff_robot * force_diff_robot/force_diff_robot.norm();
			}
			if(desired_force_robot.norm() > 10.0)
			{
				desired_force_robot *= 10.0/desired_force_robot.norm();
			}

			// Vector3d force_error = posori_task->_sigma_force * (sensed_force_moment_world_frame.head(3) - desired_force_robot);
			// integrated_force_error += force_error * 0.001;

			// desired_force_robot -= posori_task->_sigma_force * (kp_force * force_error + ki_force * integrated_force_error);
			// posori_task->_desired_force = filter_force_command_robot->update(desired_force_robot);
			posori_task->_desired_force = desired_force_robot;

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
			robot_position_global = posori_task->_current_position;

			// if(force_space_dimension != previous_force_space_dimension && force_space_dimension != 0)
			// {
			// 	contact_transition_counter = 150;
			// 	k_vir_haptic = 0;
			// }
			// else
			// {
			// 	contact_transition_counter--;
			// }

			// if(contact_transition_counter > 0)
			// {
			// 	k_vir_haptic += 1.0/150.0 * k_vir_haptic_goal;
			// 	// teleop_task->_commanded_force_device = - 0.9 * _max_damping_device0[0] * delayed_sigma_force * teleop_task->_current_trans_velocity_device;
			// 	// teleop_task->_commanded_force_device = -15.0 * teleop_task->_current_trans_velocity_device;
			// }
			// else
			// {
			// 	contact_transition_counter = 0;
			// 	k_vir_haptic = k_vir_haptic_goal;
			// }

			haptic_proxy_diff = teleop_task->_Rotation_Matrix_DeviceToRobot * (desired_position - delayed_robot_position);
			// haptic_proxy_diff = (desired_position - delayed_robot_position);
			// Vector3d deisred_force_haptic = delayed_sigma_force * (-k_vir_haptic * haptic_proxy_diff - kv_haptic * teleop_task->_current_trans_velocity_device);
			Vector3d deisred_force_haptic = - delayed_sigma_force * k_vir_haptic * haptic_proxy_diff;
			if(deisred_force_haptic.norm() > 10.0)
			{
				deisred_force_haptic *= 10.0/deisred_force_haptic.norm();
			}
			Vector3d force_diff_haptic = deisred_force_haptic - prev_desired_force_haptic;
			if( force_diff_haptic.norm() > max_force_diff_haptic )
			{
			// 	// cout << "prev force :\n" << prev_desired_force_robot.transpose() << endl;
			// 	// cout << "new force command :\n" << force_command.transpose() << endl;
			// 	// cout << endl;
				deisred_force_haptic = prev_desired_force_haptic + max_force_diff_haptic * force_diff_haptic/force_diff_haptic.norm();
			}

			// teleop_task->_commanded_force_device = filter_force_command_haptic->update(deisred_force_haptic) - delayed_sigma_force * kv_haptic * teleop_task->_current_trans_velocity_device;
			teleop_task->_commanded_force_device = deisred_force_haptic - kv_haptic * delayed_sigma_force * teleop_task->_current_trans_velocity_device;
			// teleop_task->_commanded_force_device = filter_force_command_haptic->update(deisred_force_haptic);
			// teleop_task->_commanded_force_device = -sensed_force_moment_world_frame.head(3)/Ks - 5.0 * teleop_task->_current_trans_velocity_device;
			// teleop_task->_commanded_force_device = -delayed_sensed_force/Ks;

			// teleop_task->_commanded_force_device -= kv_haptic * delayed_sigma_force * teleop_task->_current_trans_velocity_device;

			// haptic_PO += teleop_task->_commanded_force_device.dot(teleop_task->_current_trans_velocity_device) * 0.001;

			// double vh_square = (delayed_sigma_force * teleop_task->_current_trans_velocity_device).squaredNorm();
			// double alpha_pc = 0;
			// if(haptic_PO > 0 && vh_square > 0.0001)
			// {
			// 	alpha_pc = haptic_PO / vh_square * 1000.0;
			// }
			// if(alpha_pc > 0.9 * _max_damping_device0[0] - kv_haptic)
			// {
			// 	alpha_pc = 0.9 * _max_damping_device0[0] - kv_haptic;
			// }
			// if(alpha_pc < 0)
			// {
			// 	alpha_pc = 0;
			// }

			// teleop_task->_commanded_force_device -= alpha_pc * delayed_sigma_force * teleop_task->_current_trans_velocity_device;
			// haptic_PO -= alpha_pc * vh_square * 0.001;


			// if(force_space_dimension > previous_force_space_dimension)
			// {
			// 	contact_transition_counter = 50;
			// }
			// else
			// {
			// 	contact_transition_counter--;
			// }

			// if(contact_transition_counter > 0)
			// {
			// 	teleop_task->_commanded_force_device = -0.9 * _max_damping_device0[0] * delayed_sigma_force * teleop_task->_current_trans_velocity_device;
			// 	// teleop_task->_commanded_force_device = -0.9 * _max_damping_device0[0] * teleop_task->_current_trans_velocity_device;
			// }
			// else
			// {
			// 	contact_transition_counter = 0;
			// }



			// teleop_task->_commanded_force_device.setZero();


			// cout << "PO haptic : " << haptic_PO << endl;
			// cout << "alpha_pc : " << alpha_pc << endl;
			// cout << endl;

			// cout << "k virtual :\n" << k_vir_haptic << endl;

			// adding_contact = false;
			// removing_contact = false;
			// 
			prev_desired_force_robot = desired_force_robot;
			prev_desired_force_haptic = deisred_force_haptic;
			previous_force_space_dimension = force_space_dimension;

		}

		// particle filter
		Matrix3d sigma_position_global = Matrix3d::Identity() - sigma_force_global;
		// pfilter_motion_control_buffer.push(sigma_position_global * posori_task->_linear_motion_control * freq_ratio_filter_control);


		pfilter_motion_control_buffer.push(-sigma_position_global * robot_proxy_diff * freq_ratio_filter_control);
		pfilter_force_control_buffer.push(-sigma_force_global * robot_proxy_diff * freq_ratio_filter_control);

		// pfilter_motion_control_buffer.push(sigma_position_global * posori_task->_Lambda_modified.block<3,3>(0,0) * posori_task->_linear_motion_control * freq_ratio_filter_control);
		// pfilter_force_control_buffer.push(sigma_force_global * posori_task->_linear_force_control * freq_ratio_filter_control);

		pfilter_sensed_velocity_buffer.push(posori_task->_current_velocity * freq_ratio_filter_control);
		pfilter_sensed_force_buffer.push(sensed_force_moment_world_frame.head(3) * freq_ratio_filter_control);

		motion_control_pfilter += pfilter_motion_control_buffer.back();
		force_control_pfilter += pfilter_force_control_buffer.back();
		measured_velocity_pfilter += pfilter_sensed_velocity_buffer.back();
		measured_force_pfilter += pfilter_sensed_force_buffer.back();

		if(pfilter_motion_control_buffer.size() > 1/freq_ratio_filter_control)
		{

			motion_control_pfilter -= pfilter_motion_control_buffer.front();
			force_control_pfilter -= pfilter_force_control_buffer.front();
			measured_velocity_pfilter -= pfilter_sensed_velocity_buffer.front();
			measured_force_pfilter -= pfilter_sensed_force_buffer.front();

			pfilter_motion_control_buffer.pop();
			pfilter_force_control_buffer.pop();
			pfilter_sensed_velocity_buffer.pop();
			pfilter_sensed_force_buffer.pop();			

		}

		// if( teleop_task->_current_position_device(2) < -0.0315)
		// {
		// 	cout << "motion control :\n" << motion_control_pfilter.transpose() << endl;
		// 	cout << "force control robot :\n" << force_control_pfilter.transpose() << endl;
		// 	cout << "force control haptic :\n" << teleop_task->_commanded_force_device.transpose() << endl;
		// 	cout << "position haptic :\n" << teleop_task->_current_position_device.transpose() << endl;
		// 	cout << "sigma force :\n" << posori_task->_sigma_force << endl;
		// 	cout << "sigma motion :\n" << posori_task->_sigma_motion << endl;
		// 	// cout << "lambda motion control :\n" << (posori_task->_Lambda * motion_control_pfilter).transpose() << endl;
		// 	cout << endl;
		// }

		// write control torques
		redis_client.executeWriteCallback(0);

		// logger
		log_robot_position = posori_task->_current_position;
		log_robot_velocity = posori_task->_current_velocity;
		log_haptic_position = haptic_position_global;
		log_haptic_velocity = haptic_velocity_global;
		log_robot_force = posori_task->_desired_force;
		log_haptic_force = teleop_task->_commanded_force_device;
		log_sensed_force = -sensed_force_moment_world_frame.head(3);

		// // cout statements
		// if(controller_counter % 500 == 0)
		// {
		// 	cout << "controller counter : " << controller_counter << endl;
		// 	cout << "desired force : " << posori_task->_desired_force.transpose() << endl;
		// 	cout << endl;
		// }

		controller_counter++;



	}

	logger->stop();

	communication_thread.join();
	particle_filter_thread.join();

	//// Send zero force/torque to robot and haptic device through Redis keys ////
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0], Vector3d::Zero());
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0], Vector3d::Zero());
	redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], "0.0");

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


}

void communication()
{
	const double communication_delay_ms = 200;

	queue<Vector3d> robot_position_buffer;
	queue<Vector3d> haptic_position_buffer;
	queue<Vector3d> haptic_velocity_buffer;
	queue<Vector3d> sensed_force_buffer;
	queue<Matrix3d> sigma_force_buffer;

	// create a timer
	double communication_freq = 50.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(communication_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	unsigned long long communication_counter = 0;
	const int communication_delay_ncycles = communication_delay_ms / 1000.0 * communication_freq;
	
	while(runloop)
	{
		timer.waitForNextLoop();

		if(communication_delay_ncycles == 0)
		{
			delayed_haptic_position = haptic_position_global;
			delayed_haptic_velocity = haptic_velocity_global;
			delayed_robot_position = robot_position_global;
			delayed_sensed_force = sensed_force_global;
			delayed_sigma_force = sigma_force_global;
		}
		else
		{
			haptic_position_buffer.push(haptic_position_global);
			haptic_velocity_buffer.push(haptic_velocity_global);
			robot_position_buffer.push(robot_position_global);
			sensed_force_buffer.push(sensed_force_global);
			sigma_force_buffer.push(sigma_force_global);

			if(communication_counter > communication_delay_ncycles)
			{
				delayed_haptic_position = haptic_position_buffer.front();
				delayed_haptic_velocity = haptic_velocity_buffer.front();
				delayed_robot_position = robot_position_buffer.front();
				delayed_sensed_force = sensed_force_buffer.front();
				delayed_sigma_force = sigma_force_buffer.front();

				haptic_position_buffer.pop();
				haptic_velocity_buffer.pop();
				robot_position_buffer.pop();
				sensed_force_buffer.pop();
				sigma_force_buffer.pop();

				communication_counter--;
			}

		}


		communication_counter++;
	}


}

void particle_filter()
{

	// start redis client for particles
	auto redis_client_particles = RedisClient();
	redis_client_particles.connect();

	unsigned long long pf_counter = 0;

	// create particle filter
	auto pfilter = new ForceSpaceParticleFilter_weight_mem(n_particles);

	pfilter->_mean_scatter = 0.0;
	pfilter->_std_scatter = 0.025;

	pfilter->_memory_coefficient = 0.0;

	pfilter->_coeff_friction = 0.0;

	pfilter->_F_low = 0.0;
	pfilter->_F_high = 2.0;
	pfilter->_v_low = 0.001;
	pfilter->_v_high = 0.01;

	pfilter->_F_low_add = 2.0;
	pfilter->_F_high_add = 10.0;
	pfilter->_v_low_add = 0.001;
	pfilter->_v_high_add = 0.005;


	Vector3d evals = Vector3d::Zero();
	Matrix3d evecs = Matrix3d::Identity();

	// create a timer
	double pfilter_freq = 15.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(pfilter_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while(runloop)
	{
		timer.waitForNextLoop();


		pfilter->_force_space_dimension = force_space_dimension;
		pfilter->_force_axis = force_axis;
		pfilter->_motion_axis = motion_axis;


		pfilter->update(motion_control_pfilter, force_control_pfilter, measured_velocity_pfilter, measured_force_pfilter);
		pfilter->computePCA(evals, evecs);

		Vector3d evals_no_scaling = evals;


		double threshold_up = 10.0;
		double threshold_down = 0.05;

		if(evals(2) > 10.0)
		{
			evals /= evals(2);
			threshold_up = 0.90;
		}


		if(force_space_dimension == 1)
		{
			threshold_up = 0.50;
			threshold_down = 0.05;
		}
		if(force_space_dimension == 2)
		{
			threshold_up = 0.50;
			threshold_down = 0.05;
		}
		if(force_space_dimension == 3)
		{
			threshold_up = 0.50;
			threshold_down = 0.05;
		}


		double force_space_dimension_up = 0;
		double force_space_dimension_down = 0;
		for(int i=0 ; i<3 ; i++)
		{
			if(evals(i) > threshold_up)
			{
				force_space_dimension_up++;
			}
			if(evals(i) > threshold_down)
			{
				force_space_dimension_down++;
			}
		}






		// if(evals.sum() > 1)
		// {
		// 	evals /= evals.sum();
		// }


		// double force_space_dimension_up = 0;
		// double force_space_dimension_down = 0;
		// for(int i=0 ; i<3 ; i++)
		// {
		// 	if(evals(i) > 0.25)
		// 	{
		// 		force_space_dimension_up++;
		// 	}
		// 	if(evals(i) > 0.05)
		// 	{
		// 		force_space_dimension_down++;
		// 	}
		// }







		if(force_space_dimension_up == force_space_dimension_down)
		{
			force_space_dimension = force_space_dimension_down;
		}
		else
		{
			if(force_space_dimension >= force_space_dimension_down)
			{
				force_space_dimension = force_space_dimension_down;
			}
			else if(force_space_dimension <= force_space_dimension_up)
			{
				force_space_dimension = force_space_dimension_up;
			}
		}

		Vector3d force_axis_tmp = evecs.col(2);
		Vector3d motion_axis_tmp = evecs.col(0);


		if(force_axis_tmp(2) < -0.01)
		{
			force_axis = -force_axis_tmp;
		}
		else if(force_axis_tmp(1) < -0.01)
		{
			force_axis = -force_axis_tmp;
		}
		else if(force_axis_tmp(0) < -0.01)
		{
			force_axis = -force_axis_tmp;
		}
		else
		{
			force_axis = force_axis_tmp;
		}

		if(motion_axis_tmp(1) < -0.01)
		{
			motion_axis = -motion_axis_tmp;
		}
		else if(motion_axis_tmp(2) < -0.01)
		{
			motion_axis = -motion_axis_tmp;
		}
		else if(motion_axis_tmp(0) < -0.01)
		{
			motion_axis = -motion_axis_tmp;
		}
		else
		{
			motion_axis = motion_axis_tmp;
		}

		// logger
		log_eigenvalues = evals;
		log_eigenvector_0 = evecs.col(0);
		log_eigenvector_1 = evecs.col(1);
		log_eigenvector_2 = evecs.col(2);

		log_force_axis = force_axis;
		log_motion_axis = motion_axis;

		log_force_space_dimension(0) = force_space_dimension;


		Vector3d prospective_particle = Vector3d::Zero();
		if(motion_control_pfilter.norm() > 0.001)
		{
			prospective_particle = motion_control_pfilter.normalized();
		}
		double for_weight_pp_add = pfilter->wf_pw(prospective_particle, measured_force_pfilter, pfilter->_F_low_add, pfilter->_F_high_add);
		double vel_weight_pp_add = pfilter->wv_pw(prospective_particle, measured_velocity_pfilter, pfilter->_v_low_add, pfilter->_v_high_add);
		double for_weight_pp = pfilter->wf_pw(prospective_particle, measured_force_pfilter, pfilter->_F_low, pfilter->_F_high);
		double vel_weight_pp = pfilter->wv_pw(prospective_particle, measured_velocity_pfilter, pfilter->_v_low, pfilter->_v_high);

		Vector3d center_particle = Vector3d::Zero();
		double for_weight_cp_add = pfilter->wf_pw(center_particle, measured_force_pfilter, pfilter->_F_low_add, pfilter->_F_high_add);
		double vel_weight_cp_add = pfilter->wv_pw(center_particle, measured_velocity_pfilter, pfilter->_v_low_add, pfilter->_v_high_add);
		double for_weight_cp = pfilter->wf_pw(center_particle, measured_force_pfilter, pfilter->_F_low, pfilter->_F_high);
		double vel_weight_cp = pfilter->wv_pw(center_particle, measured_velocity_pfilter, pfilter->_v_low, pfilter->_v_high);

		Vector3d down_particle = Vector3d(0, 0, -1);
		double for_weight_dp_add = pfilter->wf_pw(down_particle, measured_force_pfilter, pfilter->_F_low_add, pfilter->_F_high_add);
		double vel_weight_dp_add = pfilter->wv_pw(down_particle, measured_velocity_pfilter, pfilter->_v_low_add, pfilter->_v_high_add);
		double for_weight_dp = pfilter->wf_pw(down_particle, measured_force_pfilter, pfilter->_F_low, pfilter->_F_high);
		double vel_weight_dp = pfilter->wv_pw(down_particle, measured_velocity_pfilter, pfilter->_v_low, pfilter->_v_high);

		Vector3d right_particle = Vector3d(0, 1, 0);
		double for_weight_rp_add = pfilter->wf_pw(right_particle, measured_force_pfilter, pfilter->_F_low_add, pfilter->_F_high_add);
		double vel_weight_rp_add = pfilter->wv_pw(right_particle, measured_velocity_pfilter, pfilter->_v_low_add, pfilter->_v_high_add);
		double for_weight_rp = pfilter->wf_pw(right_particle, measured_force_pfilter, pfilter->_F_low, pfilter->_F_high);
		double vel_weight_rp = pfilter->wv_pw(right_particle, measured_velocity_pfilter, pfilter->_v_low, pfilter->_v_high);


		if(previous_force_space_dimension != force_space_dimension)
		{
			cout << "********************************" << endl;
			cout << "pf coutner : " << pf_counter << endl;
			cout << "contact space dim : " << force_space_dimension << endl;
			cout << "previous contact space dim : " << previous_force_space_dimension << endl;
			cout << "eigenvectors :\n" << evecs << endl;
			cout << "eigenvalues :\n" << evals.transpose() << endl;
			cout << "eigenvalues before scaling :\n" << evals_no_scaling.transpose() << endl;
			cout << "pfilter motion control : " << motion_control_pfilter.transpose() << endl;
			cout << "pfilter force control : " << force_control_pfilter.transpose() << endl;
			cout << "pfilter meas vel : " << measured_velocity_pfilter.transpose() << endl;
			cout << "pfilter meas force : " << measured_force_pfilter.transpose() << endl;
			cout << "prospective particle : " << prospective_particle.transpose() << endl; 
			cout << "force weight pp add : " << for_weight_pp_add << endl; 
			cout << "velocity weight pp_add : " << vel_weight_pp_add << endl;
			cout << "prob add particle : " << vel_weight_pp_add*for_weight_pp_add << endl;
			cout << "force weight cp : " << for_weight_cp << endl; 
			cout << "velocity weight cp : " << vel_weight_cp << endl; 
			cout << "force weight dp : " << for_weight_dp << endl; 
			cout << "velocity weight dp : " << vel_weight_dp << endl; 
			cout << "force weight rp : " << for_weight_rp << endl; 
			cout << "velocity weight rp : " << vel_weight_rp << endl; 
			cout << endl;
		}

		if(pf_counter % 50 == 0)
		{
			cout << "-----------------------------------" << endl;
			cout << "-----------------------------------" << endl;
			cout << "pf coutner : " << pf_counter << endl;
			cout << "contact space dim : " << force_space_dimension << endl;
			cout << "previous contact space dim : " << previous_force_space_dimension << endl;
			cout << "eigenvectors :\n" << evecs << endl;
			cout << "eigenvalues :\n" << evals.transpose() << endl;
			cout << "eigenvalues before scaling :\n" << evals_no_scaling.transpose() << endl;
			cout << "pfilter motion control : " << motion_control_pfilter.transpose() << endl;
			cout << "pfilter force control : " << force_control_pfilter.transpose() << endl;
			cout << "pfilter meas vel : " << measured_velocity_pfilter.transpose() << endl;
			cout << "pfilter meas force : " << measured_force_pfilter.transpose() << endl;
			cout << "force weight pp_add : " << for_weight_pp_add << endl; 
			cout << "velocity weight pp_add : " << vel_weight_pp_add << endl;
			cout << "prob add particle : " << vel_weight_pp_add*for_weight_pp_add << endl;
			cout << "force weight cp : " << for_weight_cp << endl; 
			cout << "velocity weight cp : " << vel_weight_cp << endl; 
			cout << "force weight dp : " << for_weight_dp << endl; 
			cout << "velocity weight dp : " << vel_weight_dp << endl; 
			cout << "force weight rp : " << for_weight_rp << endl; 
			cout << "velocity weight rp : " << vel_weight_rp << endl; 
			cout << endl;			
		}



		// previous_force_space_dimension = force_space_dimension;

		for(int i=0 ; i<n_particles ; i++)
		{
			particle_positions_to_redis.col(i) = pfilter->_particles[i];
		}
		redis_client_particles.setEigenMatrixJSON(PARTICLE_POSITIONS_KEY, particle_positions_to_redis);

		pf_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Particle Filter Loop run time  : " << end_time << " seconds\n";
	std::cout << "Particle Filter Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Particle Filter Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}
