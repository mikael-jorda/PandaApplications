// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/two_arm_panda.urdf";

#define MOVE_TO_INITIAL_CONFIG      0
#define CARTESIAN_MOVE              1
#define HAPTIC_CONTROL              2

int state = MOVE_TO_INITIAL_CONFIG;

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";

// - read
const std::string TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

// - gripper
const std::string LEFT_GRIPPER_MODE_KEY  = "sai2::PandaApplication::gripper::left::mode"; // m for move and g for graps
const std::string LEFT_GRIPPER_CURRENT_WIDTH_KEY  = "sai2::PandaApplication::gripper::left::current_width";
const std::string LEFT_GRIPPER_DESIRED_WIDTH_KEY  = "sai2::PandaApplication::gripper::left::desired_width";
const std::string LEFT_GRIPPER_DESIRED_SPEED_KEY  = "sai2::PandaApplication::gripper::left::desired_speed";
const std::string LEFT_GRIPPER_DESIRED_FORCE_KEY  = "sai2::PandaApplication::gripper::left::desired_force";
const std::string RIGHT_GRIPPER_MODE_KEY  = "sai2::PandaApplication::gripper::right::mode"; // m for move and g for graps
const std::string RIGHT_GRIPPER_CURRENT_WIDTH_KEY  = "sai2::PandaApplication::gripper::right::current_width";
const std::string RIGHT_GRIPPER_DESIRED_WIDTH_KEY  = "sai2::PandaApplication::gripper::right::desired_width";
const std::string RIGHT_GRIPPER_DESIRED_SPEED_KEY  = "sai2::PandaApplication::gripper::right::desired_speed";
const std::string RIGHT_GRIPPER_DESIRED_FORCE_KEY  = "sai2::PandaApplication::gripper::right::desired_force";

unsigned long long controller_counter = 0;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();
	cout << "initial q : " << initial_q.transpose() << endl;

	// prepare controller	
	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	joint_task->_desired_position(0) += 0.2;

	// posori controller
	const string left_hand_link_name = "left_arm_link7";
	const Eigen::Vector3d left_hand_pos_in_link = Vector3d(0,0,0.2);
	const string right_hand_link_name = "right_arm_link7";
	const Eigen::Vector3d right_hand_pos_in_link = Vector3d(0,0,0.2);

	auto left_hand_posori_task = new Sai2Primitives::PosOriTask(robot, left_hand_link_name, left_hand_pos_in_link);
	VectorXd left_hand_posori_task_torques = VectorXd::Zero(dof);
	auto right_hand_posori_task = new Sai2Primitives::PosOriTask(robot, right_hand_link_name, right_hand_pos_in_link);
	VectorXd right_hand_posori_task_torques = VectorXd::Zero(dof);
	right_hand_posori_task->_kp_pos = 200.0;
	right_hand_posori_task->_kv_pos = 30.0;
	right_hand_posori_task->_kp_ori = 200.0;
	right_hand_posori_task->_kv_ori = 30.0;

	// haptic controller 
	// cDeltaDevicePtr hapticDevice0 = cDeltaDevice::create(0);
	// cDeltaDevicePtr hapticDevice1 = cDeltaDevice::create(1);
    
    auto handler = new cHapticDeviceHandler();
	// cDeltaDevicePtr hapticDevice0;
	// cDeltaDevicePtr hapticDevice1;
	// handler->getDevice(hapticDevice0, 0);
	// handler->getDevice(hapticDevice1, 1);

	auto haptic_controller_right_hand = new Sai2Primitives::OpenLoopTeleop(handler, 0, right_hand_posori_task->_current_position,
												right_hand_posori_task->_current_orientation, Eigen::Matrix3d::Identity());
	while(!haptic_controller_right_hand->device_started) 
	{
		cout << "starting right device" << endl;
	}
	haptic_controller_right_hand->GravityCompTask();


	auto haptic_controller_left_hand = new Sai2Primitives::OpenLoopTeleop(handler, 1, left_hand_posori_task->_current_position,
												left_hand_posori_task->_current_orientation, Eigen::Matrix3d::Identity());
	while(!haptic_controller_left_hand->device_started) 
	{
		cout << "starting left device" << endl;
	}
	haptic_controller_left_hand->GravityCompTask();

	bool previous_left_gripper_state = false;
	bool previous_right_gripper_state = false;
	Vector3d workspace_center_right = right_hand_posori_task->_current_position;
	Vector3d workspace_center_left = left_hand_posori_task->_current_position;

	// haptic_controller_right_hand->hapticDevice->enableForces(true);
	// haptic_controller_left_hand->hapticDevice->enableForces(true);
	haptic_controller_right_hand->EnableGripperUserSwitch();
	haptic_controller_left_hand->EnableGripperUserSwitch();

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) 
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		robot->coriolisForce(coriolis);		

		if(state == MOVE_TO_INITIAL_CONFIG)
		{
			// update tasks model
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			haptic_controller_right_hand->HomingTask();
			haptic_controller_left_hand->HomingTask();

			// if((joint_task->_desired_position - joint_task->_current_position).norm() < 1e-5 && haptic_controller_right_hand->device_homed)
			if((joint_task->_desired_position - joint_task->_current_position).norm() < 1e-5)
			{
				left_hand_posori_task->reInitializeTask();
				right_hand_posori_task->reInitializeTask();

				workspace_center_right = right_hand_posori_task->_current_position;
				workspace_center_left = left_hand_posori_task->_current_position;

				haptic_controller_right_hand->setRobotCenter(workspace_center_right, right_hand_posori_task->_current_orientation);
				haptic_controller_left_hand->setRobotCenter(workspace_center_left, left_hand_posori_task->_current_orientation);

				// left_hand_posori_task->_desired_position += Vector3d(0,-0.2,-0.21);
				// right_hand_posori_task->_desired_position += Vector3d(0,0.2,-0.13);
				// posori_task->_desired_position += Vector3d(0,0,-0.1);
				// state = CARTESIAN_MOVE;
				state = HAPTIC_CONTROL;
			}
		}
		else if(state == CARTESIAN_MOVE)
		{
			// update tasks model
			N_prec.setIdentity();
			left_hand_posori_task->updateTaskModel(N_prec);
			N_prec = left_hand_posori_task->_N;
			right_hand_posori_task->updateTaskModel(N_prec);
			N_prec = right_hand_posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			left_hand_posori_task->computeTorques(left_hand_posori_task_torques);
			right_hand_posori_task->computeTorques(right_hand_posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = left_hand_posori_task_torques + right_hand_posori_task_torques + joint_task_torques + coriolis;
		}
		else if(state == HAPTIC_CONTROL)
		{
			// update tasks model
			N_prec.setIdentity();
			left_hand_posori_task->updateTaskModel(N_prec);
			N_prec = left_hand_posori_task->_N;
			right_hand_posori_task->updateTaskModel(N_prec);
			N_prec = right_hand_posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			left_hand_posori_task->computeTorques(left_hand_posori_task_torques);
			right_hand_posori_task->computeTorques(right_hand_posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = left_hand_posori_task_torques + right_hand_posori_task_torques + joint_task_torques + coriolis;
		
			if(haptic_controller_right_hand->ReadGripperUserSwitch())
			{
				if(!previous_right_gripper_state)
				{
					haptic_controller_right_hand->setRobotCenter(workspace_center_right, right_hand_posori_task->_current_orientation);
					haptic_controller_right_hand->setDeviceCenter(Vector3d::Zero(), haptic_controller_right_hand->_rot_dev);
				}
				haptic_controller_right_hand->computeHapticCommands_Impedance(right_hand_posori_task->_desired_position,
																		right_hand_posori_task->_desired_orientation,
																		right_hand_posori_task->_current_position,
																		right_hand_posori_task->_current_orientation);
			}
			else
			{
				haptic_controller_right_hand->computeHapticCommands_Impedance_PositionOnly(right_hand_posori_task->_desired_position,
																		right_hand_posori_task->_current_position);
			}
			if(haptic_controller_left_hand->ReadGripperUserSwitch())
			{
				if(!previous_left_gripper_state)
				{
					haptic_controller_left_hand->setRobotCenter(workspace_center_left, left_hand_posori_task->_current_orientation);
					haptic_controller_left_hand->setDeviceCenter(Vector3d::Zero(), haptic_controller_left_hand->_rot_dev);
				}
				haptic_controller_left_hand->computeHapticCommands_Impedance(left_hand_posori_task->_desired_position,
																		left_hand_posori_task->_desired_orientation,
																		left_hand_posori_task->_current_position,
																		left_hand_posori_task->_current_orientation);
			}
			else
			{
				haptic_controller_left_hand->computeHapticCommands_Impedance_PositionOnly(left_hand_posori_task->_desired_position,
																		left_hand_posori_task->_current_position);
			}
		}

		// send to redis
		redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY, command_torques);

		previous_right_gripper_state = haptic_controller_right_hand->gripper_state;
		previous_left_gripper_state = haptic_controller_left_hand->gripper_state;

		controller_counter++;

	}

	delete haptic_controller_right_hand;
	delete haptic_controller_left_hand;

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
