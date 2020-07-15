// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PositionTask.h"
#include "haptic_tasks/HapticController.h"
#include "filters/ButterworthFilter.h"
#include "Logger.h"

#include "ForceSpaceParticleFilter.h"

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

const string robot_file = "./resources/sphere.urdf";
const string link_name = "end_effector"; //robot end-effector
// Set sensor frame transform in end-effector frame
Affine3d sensor_transform_in_link = Affine3d::Identity();
const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);
const Vector3d pos_in_link = Vector3d(0.0,0.0,0.0);

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

const string ROBOT_POS_KEY = "sai2::PandaApplications::18::simviz_sphere::sensors::sphere_pos";
const string ROBOT_VEL_KEY = "sai2::PandaApplications::18::simviz_sphere::sensors::sphere_vel";
const string ROBOT_COMMAND_TORQUES_KEY = "sai2::PandaApplications::18::simviz_sphere::actuators::command_torques";

const string ROBOT_SENSED_FORCE_KEY = "sai2::PandaApplications::18::simviz_sphere::sensors::sensed_force";

const string PARTICLE_POSITIONS_KEY = "sai2::PandaApplications::18::simviz_sphere::particle_positions";

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
const int n_particles = 70;
MatrixXd particle_positions_to_redis;

// const double percent_chance_contact_appears = 0.01;
const double percent_chance_contact_disapears = 0.95;
const double mean_scatter = 0.0;
const double std_scatter = 0.005;

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
Vector3d log_haptic_position = Vector3d::Zero();
Vector3d log_robot_force = Vector3d::Zero();
Vector3d log_haptic_force = Vector3d::Zero();
Vector3d log_sensed_force = Vector3d::Zero();
Vector3d log_eigenvalues = Vector3d::Zero();
Vector3d log_eigenvector_0 = Vector3d::Zero();
Vector3d log_eigenvector_1 = Vector3d::Zero();
Vector3d log_eigenvector_2 = Vector3d::Zero();

Vector3d log_force_axis = Vector3d::Zero();
Vector3d log_motion_axis = Vector3d::Zero();


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

int main() {
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
	T_workd_robot.translation() = Vector3d(0, 0, 0.15);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_workd_robot);

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	int state = INIT;
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	Vector3d x_init = joint_task->_current_position;
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = false;

	// position task
	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);
	VectorXd pos_task_torques = VectorXd::Zero(dof);
	pos_task->_use_interpolation_flag = false;
	pos_task->_use_velocity_saturation_flag = false;

	pos_task->_kp = 100.0;
	pos_task->_kv = 20.0;

	double k_vir_robot = 300.0;
	const double k_vir_haptic_goal = 300.0;
	double k_vir_haptic = k_vir_haptic_goal;
	// Matrix3d k_vir_haptic = k_vir_haptic_goal * Matrix3d::Ones();
	Vector3d robot_pos_error = Vector3d::Zero();
	Vector3d haptic_pos_error = Vector3d::Zero();

	double haptic_PO = 0;

	const double max_force_diff_robot = 0.5;
	const double max_force_diff_haptic = 0.5;
	Vector3d prev_force_command_robot = Vector3d::Zero();
	Vector3d prev_force_command_haptic = Vector3d::Zero();

	auto filter_force_command_robot = new ButterworthFilter(3,0.05);
	auto filter_force_command_haptic = new ButterworthFilter(3,0.05);

	double kp_force = 0.0;
	double ki_force = 0.0;
	Vector3d integrated_force_error = Vector3d::Zero();

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

	double kv_haptic = 10.0;

	int contact_transition_counter = 50;

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

    redis_client.addEigenToReadCallback(0, ROBOT_POS_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, ROBOT_VEL_KEY, robot->_dq);

    VectorXd sensed_force_moment = VectorXd::Zero(6);
    redis_client.addEigenToReadCallback(0, ROBOT_SENSED_FORCE_KEY, sensed_force_moment);


	// Objects to write to redis
	//write haptic commands
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_FORCE_KEYS[0], teleop_task->_commanded_force_device);
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_TORQUE_KEYS[0], teleop_task->_commanded_torque_device);
	redis_client.addDoubleToWriteCallback(0, DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], teleop_task->_commanded_gripper_force_device);

	redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

	// logger
	string folder = "../../13-LocallySeparatedHapticControl/data_logging/data/";
	string timestamp = currentDateTime();
	string prefix = "data";
	string suffix = ".csv";
	string filename = folder + prefix + "_" + timestamp + suffix;
	auto logger = new Logging::Logger(1000, filename);
	
	logger->addVectorToLog(&log_robot_position, "robot_position");
	logger->addVectorToLog(&log_haptic_position, "haptic_position");
	logger->addVectorToLog(&log_robot_force, "robot_force");
	logger->addVectorToLog(&log_haptic_force, "haptic_force");
	logger->addVectorToLog(&log_sensed_force, "sensed_force");
	logger->addVectorToLog(&log_eigenvalues, "eigenvalues");
	logger->addVectorToLog(&log_eigenvector_0, "eigenvector_0");
	logger->addVectorToLog(&log_eigenvector_1, "eigenvector_1");
	logger->addVectorToLog(&log_eigenvector_2, "eigenvector_2");

	logger->addVectorToLog(&log_force_axis, "force_axis");
	logger->addVectorToLog(&log_motion_axis, "motion_axis");

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
		robot->updateModel();

		N_prec.setIdentity(dof,dof);
		pos_task->updateTaskModel(N_prec);

		teleop_task->UseGripperAsSwitch();
		gripper_state_prev = gripper_state;
		gripper_state = teleop_task->gripper_state;


		if(state == INIT)
		{

			pos_task->computeTorques(pos_task_torques);
			command_torques = pos_task_torques;

  			// compute homing haptic device
  			teleop_task->HomingTask();

			// if((pos_task->_desired_position - pos_task->_current_position).norm() < 0.2)
			if(teleop_task->device_homed && gripper_state && (pos_task->_desired_position - pos_task->_current_position).norm() < 0.2)
			{
				// Reinitialize controllers
				pos_task->reInitializeTask();
				teleop_task->reInitializeTask();

				pos_task->_kp = 200.0;
				pos_task->_kv = 30.0;

				state = CONTROL;
			}
		}

		else if(state == CONTROL)
		{

			if(force_space_dimension == 1)
			{
				pos_task->setForceAxis(force_axis);
			}
			else if(force_space_dimension == 2)
			{
				pos_task->setMotionAxis(motion_axis);
			}
			else if(force_space_dimension == 3)
			{
				pos_task->setFullForceControl();
			}
			else
			{
				pos_task->setFullMotionControl();
			}
			sigma_force_global = pos_task->_sigma_force;


			Vector3d desired_position = Vector3d::Zero();
			teleop_task->computeHapticCommands3d(desired_position);
			haptic_position_global = desired_position;

			haptic_velocity_global = teleop_task->_current_trans_velocity_device_RobFrame;
			// Vector3d desired_velocity = teleop_task->_current_trans_velocity_device_RobFrame;
			// desired_position = teleop_task->_current_position_device;

			pos_task->_desired_position = delayed_haptic_position;
			// pos_task->_desired_velocity = delayed_haptic_velocity;

			robot_pos_error = pos_task->_current_position - pos_task->_desired_position;

			Vector3d force_command_robot = pos_task->_sigma_force * k_vir_robot * robot_pos_error;
			Vector3d force_diff_robot = force_command_robot - prev_force_command_robot;
			// if( force_diff_robot.norm() > max_force_diff_robot )
			// {
			// 	// cout << "prev force :\n" << prev_force_command_robot.transpose() << endl;
			// 	// cout << "new force command :\n" << force_command.transpose() << endl;
			// 	// cout << endl;
			// 	force_command_robot = prev_force_command_robot + max_force_diff_robot * force_diff_robot/force_diff_robot.norm();
			// }
			if(force_command_robot.norm() > 10.0)
			{
				force_command_robot *= 10.0/force_command_robot.norm();
			}

			Vector3d force_error = pos_task->_sigma_force * (sensed_force_moment.head(3) - force_command_robot);
			integrated_force_error += force_error * 0.001;

			force_command_robot -= pos_task->_sigma_force * (kp_force * force_error + ki_force * integrated_force_error);
			pos_task->_desired_force = filter_force_command_robot->update(-force_command_robot);

			pos_task->computeTorques(pos_task_torques);
			command_torques = pos_task_torques;
			robot_position_global = pos_task->_current_position;

			haptic_pos_error = delayed_robot_position - desired_position;
			Vector3d force_command_haptic = delayed_sigma_force * (k_vir_robot * haptic_pos_error - kv_haptic * teleop_task->_current_trans_velocity_device);
			if(force_command_haptic.norm() > 10.0)
			{
				force_command_haptic *= 10.0/force_command_haptic.norm();
			}
			Vector3d force_diff_haptic = force_command_haptic - prev_force_command_haptic;
			// if( force_diff_haptic.norm() > max_force_diff_haptic )
			// {
			// 	// cout << "prev force :\n" << prev_force_command_robot.transpose() << endl;
			// 	// cout << "new force command :\n" << force_command.transpose() << endl;
			// 	// cout << endl;
				// force_command_haptic = prev_force_command_haptic + max_force_diff_haptic * force_diff_haptic/force_diff_haptic.norm();
			// }

			// teleop_task->_commanded_force_device = filter_force_command_haptic->update(force_command_haptic) - delayed_sigma_force * kv_haptic * teleop_task->_current_trans_velocity_device;
			// teleop_task->_commanded_force_device = force_command_haptic;
			teleop_task->_commanded_force_device = filter_force_command_haptic->update(force_command_haptic);
			// teleop_task->_commanded_force_device = -sensed_force_moment.head(3)/Ks - 5.0 * teleop_task->_current_trans_velocity_device;
			// teleop_task->_commanded_force_device = -delayed_sensed_force/Ks;

			// teleop_task->_commanded_force_device -= kv_haptic * delayed_sigma_force * teleop_task->_current_trans_velocity_device;

			if(force_space_dimension != previous_force_space_dimension && force_space_dimension != 0)
			{
				contact_transition_counter = 50;
			}
			else
			{
				contact_transition_counter--;
			}

			if(contact_transition_counter > 0)
			{
				teleop_task->_commanded_force_device = -15.0 * delayed_sigma_force * teleop_task->_current_trans_velocity_device;
				// teleop_task->_commanded_force_device = -15.0 * teleop_task->_current_trans_velocity_device;
			}
			else
			{
				contact_transition_counter = 0;
			}


			// cout << "PO haptic : " << haptic_PO << endl;
			// cout << "kv_haptic : " << kv_haptic << endl;
			// cout << endl;

			// cout << "k virtual :\n" << k_vir_haptic << endl;

			// previous_force_space_dimension = force_space_dimension;
			// adding_contact = false;
			// removing_contact = false;
			// 
			prev_force_command_robot = force_command_robot;
			prev_force_command_haptic = teleop_task->_commanded_force_device;

		}

		// particle filter
		motion_control_pfilter = pos_task->_sigma_motion * pos_task->_motion_control;
		// motion_control_pfilter += pos_task->_sigma_motion * pos_task->_motion_control / control_loop_freq;
		// motion_control_pfilter = pos_task->_sigma_motion * motion_control_pfilter;
		force_control_pfilter = pos_task->_sigma_force * pos_task->_force_control;
		measured_velocity_pfilter = pos_task->_current_velocity;
		measured_force_pfilter = sensed_force_moment.head(3);

		// if( teleop_task->_current_position_device(2) < -0.0315)
		// {
		// 	cout << "motion control :\n" << motion_control_pfilter.transpose() << endl;
		// 	cout << "force control robot :\n" << force_control_pfilter.transpose() << endl;
		// 	cout << "force control haptic :\n" << teleop_task->_commanded_force_device.transpose() << endl;
		// 	cout << "position haptic :\n" << teleop_task->_current_position_device.transpose() << endl;
		// 	cout << "sigma force :\n" << pos_task->_sigma_force << endl;
		// 	cout << "sigma motion :\n" << pos_task->_sigma_motion << endl;
		// 	// cout << "lambda motion control :\n" << (pos_task->_Lambda * motion_control_pfilter).transpose() << endl;
		// 	cout << endl;
		// }

		// write control torques
		redis_client.executeWriteCallback(0);

		// logger
		log_robot_position = pos_task->_current_position;
		log_haptic_position = haptic_position_global;
		log_robot_force = pos_task->_desired_force;
		log_haptic_force = teleop_task->_commanded_force_device;
		log_sensed_force = -sensed_force_moment.head(3);

		// // cout statements
		// if(controller_counter % 500 == 0)
		// {
		// 	cout << "controller counter : " << controller_counter << endl;
		// 	cout << "desired force : " << pos_task->_desired_force.transpose() << endl;
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
	const double communication_delay_ms = 0;

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
	auto pfilter = new ForceSpaceParticleFilter(n_particles);

	Vector3d evals = Vector3d::Zero();
	Matrix3d evecs = Matrix3d::Identity();

	// create a timer
	double pfilter_freq = 100.0;
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

		pfilter->update(motion_control_pfilter, force_control_pfilter, measured_velocity_pfilter, measured_force_pfilter);
		pfilter->computePCA(evals, evecs);

		if(evals.sum() > 1)
		{
			evals /= evals.sum();
		}

		double force_space_dimension_up = 0;
		double force_space_dimension_down = 0;
		for(int i=0 ; i<3 ; i++)
		{
			if(evals(i) > 0.25)
			{
				force_space_dimension_up++;
			}
			if(evals(i) > 0.05)
			{
				force_space_dimension_down++;
			}
		}

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

		// if(previous_force_space_dimension == 1 && force_space_dimension == 0)
		// {
		// 	cout << "********************************" << endl;
		// 	cout << "pf coutner : " << pf_counter << endl;
		// 	cout << "contact space dim : " << force_space_dimension << endl;
		// 	cout << "previous contact space dim : " << previous_force_space_dimension << endl;
		// 	cout << "force spce dimension up : " << force_space_dimension_up << endl;
		// 	cout << "force spce dimension down : " << force_space_dimension_down << endl;
		// 	cout << "eigenvalues :\n" << evals.transpose() << endl;
		// 	cout << "eigenvectors :\n" << evecs << endl;
		// 	// runloop = false;
		// }

		previous_force_space_dimension = force_space_dimension;

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
