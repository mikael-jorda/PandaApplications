// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "Logger.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>
#include <random>
#include <queue>

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
const Vector3d pos_in_link = Vector3d(0.0,0.0,0.0);

string JOINT_ANGLES_KEY = "sai2::PandaApplications::simviz_panda::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::PandaApplications::simviz_panda::sensors::dq";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::PandaApplications::simviz_panda::actuators::command_torques";

string ROBOT_SENSED_FORCE_KEY = "sai2::PandaApplications::simviz_panda::sensors::sensed_force";

string MASSMATRIX_KEY;
string CORIOLIS_KEY;

RedisClient redis_client;

#define FREE_SPACE 0
#define CONTACT    1
int state = FREE_SPACE;

// logger
Vector3d log_robot_position = Vector3d::Zero();
Vector3d log_robot_velocity = Vector3d::Zero();
Vector3d log_robot_desired_force = Vector3d::Zero();
Vector3d log_robot_force_control = Vector3d::Zero();
Vector3d log_sensed_force = Vector3d::Zero();
Vector3d log_vc = Vector3d::Zero();
Vector3d log_command_force = Vector3d::Zero();
Vector3d log_PO_Rc_Ecorr = Vector3d::Zero();

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

// const bool flag_simulation = false;
const bool flag_simulation = true;

int main(int argc, char *argv[]) {

	double kp_f_arg = 0.7;
	double ki_f_arg = 1.5;
	if(argc == 3)
	{
		kp_f_arg = stod(argv[1]);
		kp_f_arg = stod(argv[2]);
	}


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

	// load robots
	Affine3d T_workd_robot = Affine3d::Identity();
	T_workd_robot.translation() = Vector3d(0, 0, 0);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_workd_robot);

	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	// posori task
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	Vector3d x_init = posori_task->_current_position;
	posori_task->_desired_position(2) -= 0.1;

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_use_interpolation_flag = true;

	posori_task->_otg->setMaxLinearVelocity(0.10);
	posori_task->_otg->setMaxLinearAcceleration(10.0);
	posori_task->_otg->setMaxLinearJerk(50.0);

	posori_task->_kp_pos = 500.0;
	posori_task->_kv_pos = 40.0;

	posori_task->_kp_ori = 600.0;
	posori_task->_kv_ori = 38.0;

	sensor_transform_in_link.translation() = sensor_pos_in_link;
	posori_task->setForceSensorFrame(link_name, sensor_transform_in_link);

	// posori_task->setOpenLoopForceControl();
	posori_task->setClosedLoopForceControl();
	bool passivity = true;
	// bool passivity = false;
	
	posori_task->_k_ff = 0.95;
	// posori_task->_k_ff = 1.0;

	posori_task->_kp_force = kp_f_arg;
	posori_task->_ki_force = ki_f_arg;
	posori_task->_kv_force = 20.0;

	// cout << kp_f_arg << "\t" << ki_f_arg << endl;

	VectorXd sensed_force_moment_local_frame = VectorXd::Zero(6);
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

	// setup redis keys to be updated with the callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// Objects to read from redis
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

    redis_client.addEigenToReadCallback(0, ROBOT_SENSED_FORCE_KEY, sensed_force_moment_local_frame);

    MatrixXd mass_from_robot = MatrixXd::Identity(dof,dof);
    VectorXd coriolis_from_robot = VectorXd::Zero(dof);
	if(!flag_simulation)
	{
		redis_client.addEigenToReadCallback(0, MASSMATRIX_KEY, mass_from_robot);
		redis_client.addEigenToReadCallback(0, CORIOLIS_KEY, coriolis_from_robot);
	}

	// Objects to write to redis
	redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

	// logger
	string folder = "../../21-passivity_new_experiments/data_logging/data/";
	string timestamp = currentDateTime();
	string prefix = "data";
	string suffix = ".csv";
	string filename = folder + prefix + "_" + timestamp + suffix;
	auto logger = new Logging::Logger(1000, filename);
	
	logger->addVectorToLog(&log_robot_position, "robot_position");
	logger->addVectorToLog(&log_robot_velocity, "robot_velocity");
	logger->addVectorToLog(&log_robot_desired_force, "robot_deisred_force");
	logger->addVectorToLog(&log_robot_force_control, "robot_force_control");
	logger->addVectorToLog(&log_sensed_force, "sensed_force");
	logger->addVectorToLog(&log_vc, "vc");
	logger->addVectorToLog(&log_command_force, "command_force");
	logger->addVectorToLog(&log_PO_Rc_Ecorr, "PO_Rc_Ecorr");
	logger->start();

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

	runloop = true;
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

		// update forces for posori task
		posori_task->updateSensedForceAndMoment(sensed_force_moment_local_frame.head(3), sensed_force_moment_local_frame.tail(3));

		N_prec.setIdentity(dof,dof);

		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;

		joint_task->updateTaskModel(N_prec);

		if(state == FREE_SPACE)
		{
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques + posori_task_torques + coriolis;

			// if((posori_task->_desired_position - posori_task->_current_position).norm() < 0.2)
			if(posori_task->_sensed_force(2) < -5)
			{
				// Reinitialize controllers
				posori_task->reInitializeTask();
				joint_task->reInitializeTask();

				posori_task->_desired_position = posori_task->_current_position;

				posori_task->setClosedLoopForceControl();
				posori_task->_passivity_enabled = passivity;
				posori_task->setForceAxis(Vector3d::UnitZ());
				posori_task->_desired_force = Vector3d(0,0,-10);

				state = CONTACT;
			}
		}

		else if(state == CONTACT)
		{
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;

			if(controller_counter % 5000 == 0)
			{
				posori_task->_desired_position(0) += 0.1;
				posori_task->_desired_position(1) += 0.1;
			}
			if(controller_counter % 5000 == 2500)
			{
				posori_task->_desired_position(0) -= 0.1;
				posori_task->_desired_position(1) -= 0.1;
			}
		}

		// write control torques
		redis_client.executeWriteCallback(0);

		// logger
		log_robot_position = posori_task->_current_position;
		log_robot_velocity = posori_task->_current_velocity;
		log_robot_desired_force = posori_task->_desired_force;
		log_robot_force_control = posori_task->_linear_force_control;
		log_sensed_force = posori_task->_sensed_force;
		log_vc = posori_task->_vc;
		log_command_force = posori_task->_task_force.head(3);
		log_PO_Rc_Ecorr << posori_task->_passivity_observer_force, posori_task->_Rc, posori_task->_E_correction_force;

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

	//// Send zero force/torque to robot and haptic device through Redis keys ////
	command_torques.setZero();
	redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
