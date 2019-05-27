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

#define GOTO_GRASP                    0
#define HAPTIC_CONTROL_1              1
#define GO_UP                         2
#define HAPTIC_CONTROL_2              3

int state = GOTO_GRASP;

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

const string STATE_CHANGE_KEY = "state_change";

unsigned long long controller_counter = 0;

// function to update model at a slower rate
void updateModelThreadRun(shared_ptr<Sai2Model::Sai2Model> robot, 
		shared_ptr<Sai2Primitives::JointTask> joint_task, 
		shared_ptr<Sai2Primitives::PosOriTask> left_hand_posori_task,
		shared_ptr<Sai2Primitives::PosOriTask> right_hand_posori_task);

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();
	// cout << "initial q : " << initial_q.transpose() << endl;

	// prepare controller	
	int dof = robot->dof();
	// MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);

	// joint task
	auto joint_task = make_shared<Sai2Primitives::JointTask>(robot.get());
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	// joint_task->_desired_position(0) += 0.2;

	// posori controller
	const string left_hand_link_name = "left_arm_link7";
	const Eigen::Vector3d left_hand_pos_in_link = Vector3d(0,0,0.2);
	const string right_hand_link_name = "right_arm_link7";
	const Eigen::Vector3d right_hand_pos_in_link = Vector3d(0,0,0.2);

	auto left_hand_posori_task = make_shared<Sai2Primitives::PosOriTask>(robot.get(), left_hand_link_name, left_hand_pos_in_link);
	VectorXd left_hand_posori_task_torques = VectorXd::Zero(dof);
	auto right_hand_posori_task = make_shared<Sai2Primitives::PosOriTask>(robot.get(), right_hand_link_name, right_hand_pos_in_link);
	VectorXd right_hand_posori_task_torques = VectorXd::Zero(dof);
	right_hand_posori_task->_kp_pos = 200.0;
	right_hand_posori_task->_kv_pos = 30.0;
	right_hand_posori_task->_kp_ori = 200.0;
	right_hand_posori_task->_kv_ori = 30.0;

	// cout << right_hand_posori_task->_desired_position.transpose() << endl << endl;
	// left_hand_posori_task->_desired_position = Vector3d(0.62, 0.0, 0.5);

	right_hand_posori_task->_desired_position = Vector3d(0.5, 0.0, 0.45);
	left_hand_posori_task->_desired_position = Vector3d(0.40, 0.015, 0.45);

    auto handler = new cHapticDeviceHandler();

	auto haptic_controller_right_hand = new Sai2Primitives::OpenLoopTeleop(handler, 0, right_hand_posori_task->_current_position,
												right_hand_posori_task->_current_orientation, Eigen::Matrix3d::Identity());
	auto haptic_controller_left_hand = new Sai2Primitives::OpenLoopTeleop(handler, 1, left_hand_posori_task->_current_position,
												left_hand_posori_task->_current_orientation, Eigen::Matrix3d::Identity());

	Matrix3d R_haptic_robot = AngleAxisd(M_PI,Vector3d::UnitZ()).toRotationMatrix();
	haptic_controller_left_hand->setDeviceRobotRotation(R_haptic_robot);
	haptic_controller_right_hand->setDeviceRobotRotation(R_haptic_robot);

	// haptic_controller_left_hand->haptic_feedback_from_proxy = true;
	// haptic_controller_right_hand->haptic_feedback_from_proxy = true;
	haptic_controller_left_hand->_send_haptic_feedback = true;
	haptic_controller_right_hand->_send_haptic_feedback = true;

	Matrix3d reduction_force = Vector3d(1,1,1).asDiagonal();
	Matrix3d reduction_moment = Vector3d(0.05,0.05,0.05).asDiagonal();
	haptic_controller_left_hand->setForceFeedbackCtrlGains(200, 15, 15, 0.05, reduction_force, reduction_moment);
	haptic_controller_right_hand->setForceFeedbackCtrlGains(200, 15, 15, 0.05, reduction_force, reduction_moment);

	haptic_controller_left_hand->initializeSigmaDevice();
	haptic_controller_right_hand->initializeSigmaDevice();

	haptic_controller_left_hand->GravityCompTask();
	haptic_controller_right_hand->GravityCompTask();

	haptic_controller_left_hand->setScalingFactors(2, 1);
	haptic_controller_right_hand->setScalingFactors(2, 1);

	bool previous_left_gripper_state = false;
	bool previous_right_gripper_state = false;
	Vector3d workspace_center_left = left_hand_posori_task->_current_position;
	Vector3d workspace_center_right = right_hand_posori_task->_current_position;

	haptic_controller_right_hand->EnableGripperUserSwitch();
	haptic_controller_left_hand->EnableGripperUserSwitch();

	redis_client.set(STATE_CHANGE_KEY, to_string(0));

	// start model update thread
	thread model_update_thread(updateModelThreadRun, robot, joint_task, left_hand_posori_task, right_hand_posori_task);

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
		// robot->updateModel();

		if(state == GOTO_GRASP)
		{
			// update tasks model
			// joint_task->updateTaskModel(N_prec);

			// compute torques
			left_hand_posori_task->computeTorques(left_hand_posori_task_torques);
			right_hand_posori_task->computeTorques(right_hand_posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = left_hand_posori_task_torques + right_hand_posori_task_torques + joint_task_torques;

			haptic_controller_right_hand->HomingTask();
			haptic_controller_left_hand->HomingTask();

			// if((joint_task->_desired_position - joint_task->_current_position).norm() < 1e-5 && haptic_controller_right_hand->device_homed)
			if((left_hand_posori_task->_desired_position - left_hand_posori_task->_current_position).norm() < 1e-5)
				// && (right_hand_posori_task->_desired_position - right_hand_posori_task->_current_position).norm() < 1e-5 )
			{
				// left_hand_posori_task->reInitializeTask();
				// right_hand_posori_task->reInitializeTask();

				// redis_client.set(RIGHT_GRIPPER_MODE_KEY, "g");
				// redis_client.set(LEFT_GRIPPER_MODE_KEY, "g");
				// redis_client.set(RIGHT_GRIPPER_DESIRED_FORCE_KEY, "50.0");
				// redis_client.set(LEFT_GRIPPER_DESIRED_FORCE_KEY, "50.0");
				// redis_client.set(RIGHT_GRIPPER_DESIRED_WIDTH_KEY, "0.0");
				redis_client.set(LEFT_GRIPPER_DESIRED_WIDTH_KEY, "0.0");

				workspace_center_right = right_hand_posori_task->_current_position;
				workspace_center_left = left_hand_posori_task->_current_position;

				haptic_controller_right_hand->setRobotCenter(workspace_center_right, right_hand_posori_task->_current_orientation);
				haptic_controller_left_hand->setRobotCenter(workspace_center_left, left_hand_posori_task->_current_orientation);

				right_hand_posori_task->_use_interpolation_flag = false;
				left_hand_posori_task->_use_interpolation_flag = false;

				// left_hand_posori_task->_desired_position += Vector3d(0,-0.2,-0.21);
				// right_hand_posori_task->_desired_position += Vector3d(0,0.2,-0.13);
				// posori_task->_desired_position += Vector3d(0,0,-0.1);
				// state = CARTESIAN_MOVE;
				state = HAPTIC_CONTROL_1;
			}
		}
		else if(state == HAPTIC_CONTROL_1)
		{
			// update tasks model
			// N_prec.setIdentity();
			// left_hand_posori_task->updateTaskModel(N_prec);
			// N_prec = left_hand_posori_task->_N;
			// right_hand_posori_task->updateTaskModel(N_prec);
			// N_prec = right_hand_posori_task->_N;
			// joint_task->updateTaskModel(N_prec);

			// haptic control
			haptic_controller_left_hand->updateSensedRobotPositionVelocity(left_hand_posori_task->_current_position,
																		left_hand_posori_task->_current_velocity,
																		left_hand_posori_task->_current_orientation,
																		left_hand_posori_task->_current_angular_velocity);
			haptic_controller_right_hand->updateSensedRobotPositionVelocity(right_hand_posori_task->_current_position,
																		right_hand_posori_task->_current_velocity,
																		right_hand_posori_task->_current_orientation,
																		right_hand_posori_task->_current_angular_velocity);
			if(haptic_controller_left_hand->ReadGripperUserSwitch())
			{
				// redis_client.set(LEFT_GRIPPER_DESIRED_WIDTH_KEY, "0.0");
				if(!previous_left_gripper_state)
				{
					haptic_controller_left_hand->setDeviceOrientationCenterToCurrent();
					haptic_controller_left_hand->setRobotCenter(workspace_center_left, left_hand_posori_task->_current_orientation);
				}
				haptic_controller_left_hand->computeHapticCommands6d(left_hand_posori_task->_desired_position, left_hand_posori_task->_desired_orientation);
			}
			else
			{
				// redis_client.set(LEFT_GRIPPER_DESIRED_WIDTH_KEY, "0.04");
				haptic_controller_left_hand->computeHapticCommands3d(left_hand_posori_task->_desired_position);
			}

			// haptic_controller_left_hand->computeHapticCommands6d(left_hand_posori_task->_desired_position, left_hand_posori_task->_desired_orientation);
			// haptic_controller_right_hand->computeHapticCommands6d(right_hand_posori_task->_desired_position, right_hand_posori_task->_desired_orientation);

			if(haptic_controller_right_hand->ReadGripperUserSwitch())
			{
				if(!previous_right_gripper_state)
				{
					haptic_controller_right_hand->setDeviceOrientationCenterToCurrent();
					haptic_controller_right_hand->setRobotCenter(workspace_center_right, right_hand_posori_task->_current_orientation);
				}
				haptic_controller_right_hand->computeHapticCommands6d(right_hand_posori_task->_desired_position, right_hand_posori_task->_desired_orientation);
			}
			else
			{
				haptic_controller_right_hand->computeHapticCommands3d(right_hand_posori_task->_desired_position);
			}

			// compute torques
			left_hand_posori_task->computeTorques(left_hand_posori_task_torques);
			right_hand_posori_task->computeTorques(right_hand_posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = left_hand_posori_task_torques + right_hand_posori_task_torques + joint_task_torques;

			int change_state = 0;
			change_state = stod(redis_client.get(STATE_CHANGE_KEY));
			if(change_state == 1)
			{
				joint_task->reInitializeTask();
				joint_task->_desired_position(0) += 1.8;
				state = GO_UP;
			}

		
		}
		else if(state == GO_UP)
		{
			haptic_controller_right_hand->HomingTask();
			haptic_controller_left_hand->HomingTask();

			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

 			// cout << (joint_task->_desired_position - joint_task->_current_position).norm() << endl << endl;
			if((joint_task->_desired_position(0) - joint_task->_current_position(0)) < 1e-2)
			{
				right_hand_posori_task->reInitializeTask();
				left_hand_posori_task->reInitializeTask();
			
				workspace_center_right = right_hand_posori_task->_current_position;
				workspace_center_left = left_hand_posori_task->_current_position;

				haptic_controller_right_hand->setRobotCenter(workspace_center_right, right_hand_posori_task->_current_orientation);
				haptic_controller_left_hand->setRobotCenter(workspace_center_left, left_hand_posori_task->_current_orientation);

				state = HAPTIC_CONTROL_2;
			}

		}
		else if(state == HAPTIC_CONTROL_2)
		{
			// haptic control
			haptic_controller_left_hand->updateSensedRobotPositionVelocity(left_hand_posori_task->_current_position,
																		left_hand_posori_task->_current_velocity,
																		left_hand_posori_task->_current_orientation,
																		left_hand_posori_task->_current_angular_velocity);
			haptic_controller_right_hand->updateSensedRobotPositionVelocity(right_hand_posori_task->_current_position,
																		right_hand_posori_task->_current_velocity,
																		right_hand_posori_task->_current_orientation,
																		right_hand_posori_task->_current_angular_velocity);
			if(haptic_controller_left_hand->ReadGripperUserSwitch())
			{
				if(!previous_left_gripper_state)
				{
					haptic_controller_left_hand->setDeviceOrientationCenterToCurrent();
					haptic_controller_left_hand->setRobotCenter(workspace_center_left, left_hand_posori_task->_current_orientation);
				}
				haptic_controller_left_hand->computeHapticCommands6d(left_hand_posori_task->_desired_position, left_hand_posori_task->_desired_orientation);
			}
			else
			{
				haptic_controller_left_hand->computeHapticCommands3d(left_hand_posori_task->_desired_position);
			}

			if(haptic_controller_right_hand->ReadGripperUserSwitch())
			{
				if(!previous_right_gripper_state)
				{
					haptic_controller_right_hand->setDeviceOrientationCenterToCurrent();
					haptic_controller_right_hand->setRobotCenter(workspace_center_right, right_hand_posori_task->_current_orientation);
				}
				haptic_controller_right_hand->computeHapticCommands6d(right_hand_posori_task->_desired_position, right_hand_posori_task->_desired_orientation);
			}
			else
			{
				haptic_controller_right_hand->computeHapticCommands3d(right_hand_posori_task->_desired_position);
			}

			// compute torques
			left_hand_posori_task->computeTorques(left_hand_posori_task_torques);
			right_hand_posori_task->computeTorques(right_hand_posori_task_torques);

			joint_task->computeTorques(joint_task_torques);

			command_torques = left_hand_posori_task_torques + right_hand_posori_task_torques + joint_task_torques;
		}


		// send to redis
		redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY, command_torques);

		previous_right_gripper_state = haptic_controller_right_hand->gripper_state;
		previous_left_gripper_state = haptic_controller_left_hand->gripper_state;

		controller_counter++;

	}

	model_update_thread.join();

	delete haptic_controller_right_hand;
	delete haptic_controller_left_hand;

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}

void updateModelThreadRun(shared_ptr<Sai2Model::Sai2Model> robot, 
		shared_ptr<Sai2Primitives::JointTask> joint_task, 
		shared_ptr<Sai2Primitives::PosOriTask> left_hand_posori_task,
		shared_ptr<Sai2Primitives::PosOriTask> right_hand_posori_task)
{

	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;	


	while(runloop)
	{
		timer.waitForNextLoop();

		robot->updateModel();

		if(state == GOTO_GRASP)
		{
			N_prec.setIdentity();
			left_hand_posori_task->updateTaskModel(N_prec);
			N_prec = left_hand_posori_task->_N;
			right_hand_posori_task->updateTaskModel(N_prec);
			N_prec = right_hand_posori_task->_N;
			joint_task->updateTaskModel(N_prec);
		}
		else if(state == HAPTIC_CONTROL_1)
		{
			N_prec.setIdentity();
			left_hand_posori_task->updateTaskModel(N_prec);
			N_prec = left_hand_posori_task->_N;
			right_hand_posori_task->updateTaskModel(N_prec);
			N_prec = right_hand_posori_task->_N;
			joint_task->updateTaskModel(N_prec);
		}
		else if(state == GO_UP)
		{
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
		}
		else if(state == HAPTIC_CONTROL_2)
		{
			N_prec.setIdentity();
			left_hand_posori_task->updateTaskModel(N_prec);
			N_prec = left_hand_posori_task->_N;
			right_hand_posori_task->updateTaskModel(N_prec);
			N_prec = right_hand_posori_task->_N;
			joint_task->updateTaskModel(N_prec);
		}
	}

}
