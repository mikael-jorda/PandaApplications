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

const string robot_file = "./resources/panda_no_truck_no_gripper.urdf";

#define GOTO_INIT_CONFIG              0
#define HAPTIC_CONTROL                1

int state = GOTO_INIT_CONFIG;

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

const string FIX_OBJECT_KEY = "sai2::PandaApplication::fix_object";


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
	Affine3d T_world_robot = Affine3d::Identity();
	T_world_robot.translation() << 1.3, -0.3, 7;
	T_world_robot.linear() = AngleAxisd(M_PI, Vector3d::UnitZ()).toRotationMatrix();
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false, T_world_robot);
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

	joint_task->_use_interpolation_flag = false;

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

	right_hand_posori_task->_use_interpolation_flag = false;
	left_hand_posori_task->_use_interpolation_flag = false;

	left_hand_posori_task->updateTaskModel(MatrixXd::Identity(dof,dof));
	right_hand_posori_task->updateTaskModel(MatrixXd::Identity(dof,dof));

	// cout << right_hand_posori_task->_desired_position.transpose() << endl << endl;
	// left_hand_posori_task->_desired_position = Vector3d(0.62, 0.0, 0.5);

	// right_hand_posori_task->_desired_position = Vector3d(0.5, 0.0, 0.45);
	// left_hand_posori_task->_desired_position = Vector3d(0.40, 0.015, 0.45);

	// define home positions of the panda robots
	Vector3d workspace_center_left = left_hand_posori_task->_current_position;
	Vector3d workspace_center_right = right_hand_posori_task->_current_position;

	// define home position of the haptic device
	Vector3d haptic_center_left = Vector3d::Zero();
	Vector3d haptic_center_right = Vector3d::Zero();


	// connect haptic device and prepare teleoperation controller
	auto handler = new cHapticDeviceHandler();

	//Robot Brush : Robot[0] - LEFT && Robot Palette : Robot[1] - RIGHT
	MatrixXd rot_camera = AngleAxisd(M_PI/2, Vector3d::UnitZ()).toRotationMatrix();
	Matrix3d transformDev_Rob_left = rot_camera * T_world_robot.linear(); //////////////////////////////////////////
	Matrix3d transformDev_Rob_right = rot_camera * T_world_robot.linear(); ///////////////////////////////////////////////
	auto haptic_controller_right = new Sai2Primitives::OpenLoopTeleop(handler, 0, workspace_center_right, right_hand_posori_task->_current_orientation, transformDev_Rob_right);
	auto haptic_controller_left = new Sai2Primitives::OpenLoopTeleop(handler, 1, workspace_center_left, left_hand_posori_task->_current_orientation, transformDev_Rob_left);
	//Task scaling factors
	double Ks_brush=1.5;
	double KsR_brush=1.0;
	haptic_controller_left->setScalingFactors(Ks_brush, KsR_brush);

	double _pos_gripper=0.0;


	//Task scaling factors
	double Ks_right=1.5;
	double KsR_right=1.0;
	haptic_controller_right->setScalingFactors(Ks_right, KsR_right);

	VectorXd f_task_sensed = VectorXd::Zero(6);

	Vector3d pos_proxy_model = Vector3d::Zero(); // Model position and orientation in robot frame
	Matrix3d rot_proxy_model = Matrix3d::Identity();
	Vector3d vel_rot_proxy_model = Vector3d::Zero(); // Model linear and angular velocities in robot frame
	Vector3d vel_trans_proxy_model = Vector3d::Zero();

	// Force feedback stiffness proxy parameters
	double k_pos = 3000.0;
	double d_pos = 1.0;
	double k_ori = 20.0;
	double d_ori = 0.1;
	Matrix3d Red_factor_rot = Matrix3d::Identity();
	Matrix3d Red_factor_trans = Matrix3d::Identity();
	Red_factor_rot << 1/10.0, 0.0, 0.0,
						  0.0, 1/10.0, 0.0,
						  0.0, 0.0, 1/10.0;

	Red_factor_trans << 1/2.0, 0.0, 0.0,
						  0.0, 1/2.0, 0.0,
						  0.0, 0.0, 1/2.0;
	haptic_controller_right->setForceFeedbackCtrlGains(k_pos, d_pos, k_ori, d_ori, 1/50.0, 1/1.5, Red_factor_rot, Red_factor_trans);
	
	haptic_controller_right->_filter_on = false;
	double fc_force=0.02;
	double fc_moment=0.02;
	haptic_controller_right->setFilterCutOffFreq(fc_force, fc_moment);

	haptic_controller_left->initializeSigmaDevice();
	haptic_controller_right->initializeSigmaDevice();
	haptic_controller_left->GravityCompTask();
	haptic_controller_right->GravityCompTask();

	bool gripper_state_right = false;
	haptic_controller_right->EnableGripperUserSwitch();


 //    auto handler = new cHapticDeviceHandler();

	// auto haptic_controller_right_hand = new Sai2Primitives::OpenLoopTeleop(handler, 0, right_hand_posori_task->_current_position,
	// 											right_hand_posori_task->_current_orientation, Eigen::Matrix3d::Identity());
	// auto haptic_controller_left_hand = new Sai2Primitives::OpenLoopTeleop(handler, 1, left_hand_posori_task->_current_position,
	// 											left_hand_posori_task->_current_orientation, Eigen::Matrix3d::Identity());

	// Matrix3d R_haptic_robot = AngleAxisd(M_PI,Vector3d::UnitZ()).toRotationMatrix();
	// haptic_controller_left_hand->setDeviceRobotRotation(R_haptic_robot);
	// haptic_controller_right_hand->setDeviceRobotRotation(R_haptic_robot);

	// // haptic_controller_left_hand->haptic_feedback_from_proxy = true;
	// // haptic_controller_right_hand->haptic_feedback_from_proxy = true;
	// haptic_controller_left_hand->_send_haptic_feedback = true;
	// haptic_controller_right_hand->_send_haptic_feedback = true;

	// Matrix3d reduction_force = Vector3d(1,1,1).asDiagonal();
	// Matrix3d reduction_moment = Vector3d(0.05,0.05,0.05).asDiagonal();
	// haptic_controller_left_hand->setForceFeedbackCtrlGains(200, 15, 15, 0.05, reduction_force, reduction_moment);
	// haptic_controller_right_hand->setForceFeedbackCtrlGains(200, 15, 15, 0.05, reduction_force, reduction_moment);

	// haptic_controller_left_hand->initializeSigmaDevice();
	// haptic_controller_right_hand->initializeSigmaDevice();

	// haptic_controller_left_hand->GravityCompTask();
	// haptic_controller_right_hand->GravityCompTask();

	// haptic_controller_left_hand->setScalingFactors(2, 1);
	// haptic_controller_right_hand->setScalingFactors(2, 1);

	// bool previous_left_gripper_state = false;
	// bool previous_right_gripper_state = false;
	// Vector3d workspace_center_left = left_hand_posori_task->_current_position;
	// Vector3d workspace_center_right = right_hand_posori_task->_current_position;

	// haptic_controller_right_hand->EnableGripperUserSwitch();
	// haptic_controller_left_hand->EnableGripperUserSwitch();

	// redis_client.set(STATE_CHANGE_KEY, to_string(0));

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

		if(state == GOTO_INIT_CONFIG)
		{
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques + coriolis;
			
			// compute homing haptic device
			haptic_controller_left->HomingTask();
			haptic_controller_right->HomingTask();

			if(controller_counter % 100 == 0)
			{
				cout << "robot q " << robot->_q.transpose() << endl;
				cout << "joint_task_torques : " << joint_task_torques.transpose() << endl;
				cout << endl;
			}

			// cout << "left robot error : " << (joint_task->_desired_position - joint_task->_current_position).norm() << endl;

			if((joint_task->_desired_position - joint_task->_current_position).norm() < 0.2 && 
				haptic_controller_left->device_homed &&
				haptic_controller_right->device_homed)
			{
				joint_task->reInitializeTask();

				left_hand_posori_task->reInitializeTask();
				workspace_center_left = left_hand_posori_task->_current_position;
				haptic_center_left = haptic_controller_left->_current_position_device ;

				haptic_controller_left->setRobotCenter(workspace_center_left, left_hand_posori_task->_current_orientation);
				haptic_controller_left->setDeviceCenter(haptic_center_left, haptic_controller_left->_current_rotation_device);

				right_hand_posori_task->reInitializeTask();
				workspace_center_right = right_hand_posori_task->_current_position;
				haptic_center_right = haptic_controller_right->_current_position_device ;

				haptic_controller_right->setRobotCenter(workspace_center_right, right_hand_posori_task->_current_orientation);
				haptic_controller_right->setDeviceCenter(haptic_center_right, haptic_controller_right->_current_rotation_device);
				
				cout << "ws center :" << workspace_center_left.transpose() << endl << workspace_center_right.transpose() << endl << endl << endl;

				state = HAPTIC_CONTROL;
			}
		}
		else if(state == HAPTIC_CONTROL)
		{
			
			haptic_controller_left->_send_haptic_feedback = false;
			haptic_controller_right->_send_haptic_feedback = true;

			haptic_controller_left->hapticDevice->getGripperAngleRad(_pos_gripper);
			gripper_state_right = haptic_controller_right->ReadGripperUserSwitch();

			double gripper_desired_width = _pos_gripper / 0.51 * 0.04;
			redis_client.set(LEFT_GRIPPER_DESIRED_WIDTH_KEY, to_string(gripper_desired_width));
			if(gripper_state_right)
			{
				redis_client.set(FIX_OBJECT_KEY, "1");
			}

			// if(controller_counter % 100 == 0)
			// {
			// 	cout << "gripper angle : " << _pos_gripper << endl << endl;
			// }

			//Compute haptic commands
			haptic_controller_right->_haptic_feedback_from_proxy = true; // If set to true, the force feedback is computed from a stiffness/damping proxy.
	    	haptic_controller_right->_filter_on = false; // filtering parameters are tuned in the initialization
			
			//haptic_controller_right->updateSensedForce(f_task_sensed);
			haptic_controller_right->updateSensedRobotPositionVelocity(right_hand_posori_task->_current_position, 
					right_hand_posori_task->_current_velocity, 
					right_hand_posori_task->_current_orientation, 
					right_hand_posori_task->_current_angular_velocity);

			haptic_controller_left->computeHapticCommands6d(left_hand_posori_task->_desired_position,
																   left_hand_posori_task->_desired_orientation);
			haptic_controller_right->computeHapticCommands6d(right_hand_posori_task->_desired_position,
																   right_hand_posori_task->_desired_orientation);

			// compute robot set torques
			left_hand_posori_task->computeTorques(left_hand_posori_task_torques);
			right_hand_posori_task->computeTorques(right_hand_posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			if(controller_counter % 100 == 0)
			{
				cout << "desired pos left : " << left_hand_posori_task->_desired_position.transpose() << endl;
				cout << "desired pos right : " << right_hand_posori_task->_desired_position.transpose() << endl;
				cout << "joint_task_torques : " << joint_task_torques.transpose() << endl;
				cout << endl;
			}

			// command_torques.setZero();
			// command_torques = joint_task_torques + coriolis;
			command_torques = joint_task_torques + coriolis + left_hand_posori_task_torques + right_hand_posori_task_torques;

		}
		else
		{
			command_torques.setZero();
		}
		

		// send to redis
		redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	model_update_thread.join();

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

		if(state == GOTO_INIT_CONFIG)
		{
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
		}
		else if(state == HAPTIC_CONTROL)
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
