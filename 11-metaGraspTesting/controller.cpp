// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "filters/ButterworthFilter.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "tasks/PositionTask.h"
#include "tasks/OrientationTask.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

// - gripper
std::string GRIPPER_MODE_KEY; // m for move and g for graps
std::string GRIPPER_MAX_WIDTH_KEY;
std::string GRIPPER_CURRENT_WIDTH_KEY;
std::string GRIPPER_DESIRED_WIDTH_KEY;
std::string GRIPPER_DESIRED_SPEED_KEY;
std::string GRIPPER_DESIRED_FORCE_KEY;

// gains
const string KP_JOINT_KEY = "sai2::PandaApplication::controller::kp_joint";
const string KV_JOINT_KEY = "sai2::PandaApplication::controller::kv_joint";
const string KP_POS_KEY = "sai2::PandaApplication::controller::kp_pos";
const string KV_POS_KEY = "sai2::PandaApplication::controller::kv_pos";
const string KP_ORI_KEY = "sai2::PandaApplication::controller::kp_ori";
const string KV_ORI_KEY = "sai2::PandaApplication::controller::kv_ori";

const string DESIRED_POS_KEY = "sai2::PandaApplication::controller::desired_position";
const string DESIRED_ORI_KEY = "sai2::PandaApplication::controller::desired_orientation";

const string DSIRED_POS_IN_CAMERA_FRAME_KEY = "sai2::PandaApplication::controller::desired_pos_in_camera_frame";
const string DSIRED_ROT_IN_CAMERA_FRAME_KEY = "sai2::PandaApplication::controller::desired_rot_in_camera_frame";

const string GO_TO_INITIAL_KEY = "sai2::PandaApplication::controller::go_to_initial_pos";

const string SVH_HAND_COMMAND_POSITIONS_KEY = "sai2::SVHHand_Right::position_command";
const string SVH_RECEIVED_POSITION_KEY = "sai2::PandaApplication::controller::SVH_position_command";
const string SVH_CURRENT_KEY = "sai2::SVHHand_Right::current";
const string SVH_POSITIONS_KEY = "sai2::SVHHand_Right::position";


unsigned long long controller_counter = 0;

#define GO_TO_INIT_CONFIG          0
#define WAIT_FOR_CAMERA_INFO       1
#define MOVE_ABOVE_GRASP_POSE      2
#define MOVE_TO_GRASP_POSE         3
#define GRASP                      4
#define LIFT                       5
#define DEBUG                      10

int state = GO_TO_INIT_CONFIG;


// const bool flag_simulation = false;
const bool flag_simulation = true;

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

		GRIPPER_MODE_KEY  = "sai2::PandaApplication::gripper::mode"; // m for move and g for graps
		GRIPPER_MAX_WIDTH_KEY  = "sai2::PandaApplication::gripper::max_width";
		GRIPPER_CURRENT_WIDTH_KEY  = "sai2::PandaApplication::gripper::current_width";
		GRIPPER_DESIRED_WIDTH_KEY  = "sai2::PandaApplication::gripper::desired_width";
		GRIPPER_DESIRED_SPEED_KEY  = "sai2::PandaApplication::gripper::desired_speed";
		GRIPPER_DESIRED_FORCE_KEY  = "sai2::PandaApplication::gripper::desired_force";		
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Clyde::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";		

		GRIPPER_MODE_KEY  = "sai2::FrankaPanda::Clyde::gripper::mode"; // m for move and g for graps
		GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::Clyde::gripper::max_width";
		GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::Clyde::gripper::current_width";
		GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::Clyde::gripper::desired_width";
		GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::Clyde::gripper::desired_speed";
		GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::Clyde::gripper::desired_force";
	}

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

	// Camera pose in ee frame
	string camera_link = "link7";
	Affine3d T_ee_camera = Affine3d::Identity();
	T_ee_camera.translation() = Vector3d(-0.0976, 0.0504, 0.1652);
	// T_ee_camera.translation() = Vector3d(-0.0615, 0.0904, 0.1639);
	// T_ee_camera.translation() = Vector3d(-0.0935, 0.0484, 0.1639);
	// 
	T_ee_camera.linear() << 0.714581219586, -0.699456950306, 0.0115608459457,
							0.6994185703, 0.714023515461, -0.0313700939647,
							0.0136873144587, 0.0305023504502, 0.999440974663;

	// prepare controller	
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 10.0;
	joint_task->_kv = 5.0;

	VectorXd q_init = VectorXd::Zero(dof);
	q_init << 0.739103, -0.420917, -0.390521,  -1.17678, -0.284019,  0.828127,   -1.8917;   // nominal high table
	// q_init << 0.702168, -0.473445, -0.452138,  -1.74802, -0.194966,   1.32619,  -1.70958;   // nominal small table
	// q_init << 0.699725, -0.450311, -0.460829,  -1.73831, -0.222373,   1.36854,  -1.70284;
	// q_init << 0.601074,   -0.20861,  -0.397749,   -1.54478, -0.0764859,     1.3621,  -0.290231;   // rotated
	// Position task
	const string link_name = "link7";
	// const Eigen::Vector3d pos_in_link = Vector3d(0, 0, 0.160);

	const Eigen::Vector3d p_handBase_controlPoint = Vector3d(0, 0, 0.10);
	const Eigen::Vector3d pos_in_link = Vector3d(0, 0, 0.260);
	const Eigen::Matrix3d R_ee_hand = AngleAxisd(3.0/4.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();
	// auto posori_task = new Sai2Primitives::PosOriTask(robot, camera_link, T_ee_camera.translation());
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = false;

	// posori_task->_linear_saturation_velocity = 0.15;
	// posori_task->_angular_saturation_velocity = M_PI/3;

	posori_task->_otg->setMaxLinearVelocity(0.1);
	posori_task->_otg->setMaxLinearAcceleration(1.0);
	posori_task->_otg->setMaxLinearJerk(3.0);

	posori_task->_otg->setMaxAngularVelocity(M_PI/3);
	posori_task->_otg->setMaxAngularAcceleration(M_PI);
	posori_task->_otg->setMaxAngularJerk(3*M_PI);


	Vector3d init_config_position = Vector3d::Zero();
	Matrix3d init_config_orientation = Matrix3d::Identity();

	init_config_position << 0.377303, 0.192864, 0.696588;
	init_config_orientation << -0.392588,   0.919408,  0.0237582,
							  0.919711,   0.392381,  0.0130123,
							0.00264131,  0.0269592,  -0.999633;

	redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, posori_task->_current_position);

	// posori_task->_desired_position = init_config_position;
	// posori_task->_desired_orientation = init_config_orientation;

	VectorXd posori_task_torques = VectorXd::Zero(dof);

	// posori_task->_dynamic_decoupling_type = Sai2Primitives::PosOriTask::DynamicDecouplingType::POSITION_ONLY_DYNAMIC_DECOUPLNG;

	posori_task->_kp_pos = 350.0;
	posori_task->_kv_pos = 25.0;
	// posori_task->_ki_pos = 55.0;
	posori_task->_kp_ori = 250.0;
	posori_task->_kv_ori = 20.0;
	// posori_task->_ki_ori = 55.0;


	redis_client.set(KP_POS_KEY, to_string(posori_task->_kp_pos));
	redis_client.set(KV_POS_KEY, to_string(posori_task->_kv_pos));
	redis_client.set(KP_ORI_KEY, to_string(posori_task->_kp_ori));
	redis_client.set(KV_ORI_KEY, to_string(posori_task->_kv_ori));

	redis_client.setEigenMatrixJSON(DSIRED_POS_IN_CAMERA_FRAME_KEY, Vector3d::Zero());
	redis_client.setEigenMatrixJSON(DSIRED_ROT_IN_CAMERA_FRAME_KEY, Matrix3d::Identity());
	redis_client.set(GO_TO_INITIAL_KEY, "0");

	VectorXd svh_init_config = VectorXd::Zero(9);
	svh_init_config << 0, 10, 10, 10, 30, 10, 0, 56, 10;
	redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, svh_init_config);

	VectorXd hand_position_offset = VectorXd::Zero(9);
	hand_position_offset << 0, 0, 5, 5, 0, 0, 5, 0, 0;

	int grasp_wait_counter = 0;



	vector<double> buffer_joint_task_error;
	const int buffer_size = 50;
	double cumulated_error = 0;
	for(int i=0 ; i<buffer_size ; i++)
	{
		buffer_joint_task_error.push_back(100);
		cumulated_error += buffer_joint_task_error[i];
	}


	const Vector3d index_in_hand = Vector3d(-0.0249, 0.0, 0.229);

	Matrix3d debug_ori = Matrix3d::Identity();
	debug_ori << 0, 1, 0,
				1, 0, 0,
				0, 0, -1;
	// debug_ori = AngleAxisd(0.4010693, Vector3d::UnitZ()).toRotationMatrix() * debug_ori;

	// cout << posori_task->_current_position.transpose() << endl << endl << posori_task->_current_orientation << endl << endl;

	// return 0;

	// // orientation task
	// auto ori_task = new Sai2Primitives::OrientationTask(robot, link_name, pos_in_link);

	// VectorXd ori_task_torques = VectorXd::Zero(dof);

	// ori_task->_kp = 1.0;
	// ori_task->_kv = 0.1;

	// redis_client.set(KP_ORI_KEY, to_string(ori_task->_kp));
	// redis_client.set(KV_ORI_KEY, to_string(ori_task->_kv));

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

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			robot->_M_inv = robot->_M.inverse();

			coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
		}

		// update tasks models
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		// N_prec = pos_task->_N;
		// ori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);


		if(state == DEBUG)
		{
			// read gains and desired position
			posori_task->_kp_pos = stod(redis_client.get(KP_POS_KEY)); 
			posori_task->_kv_pos = stod(redis_client.get(KV_POS_KEY)); 
			posori_task->_kp_ori = stod(redis_client.get(KP_ORI_KEY)); 
			posori_task->_kv_ori = stod(redis_client.get(KV_ORI_KEY)); 

			posori_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_POS_KEY);
			posori_task->_desired_orientation = debug_ori;

			// VectorXd SVH_positions = redis_client.getEigenMatrixJSON(SVH_POSITIONS_KEY);
			// VectorXd SVH_currents = redis_client.getEigenMatrixJSON(SVH_CURRENT_KEY);
			// VectorXd SVH_desired_positions = redis_client.getEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY);

			// double desired_current = -150.0;
			// double Ka = 0.1;

			// double index_distal_velocity_command = Ka * (SVH_currents(2) - desired_current);
			// double ring_velocity_command = Ka * (SVH_currents(6) - desired_current);

			// SVH_desired_positions(2) += 0.001 * index_distal_velocity_command;
			// SVH_desired_positions(6) += 0.001 * ring_velocity_command;

			// redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, SVH_desired_positions);


			for(int i=3 ; i<6 ; i++)
			{
				posori_task->_Lambda(i,i) += 0.1;
			}
			// posori_task->_desired_orientation = debug_ori;


			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

		}

		else if(state == GO_TO_INIT_CONFIG)
		{
			redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, svh_init_config);


			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->_desired_position = q_init;

			joint_task->_kp = 250.0;
			joint_task->_kv = 15.0;
			joint_task->_ki = 55.0;

			// posori_task->_desired_position = init_config_position;
			// posori_task->_desired_orientation = init_config_orientation;

			for(int i=4 ; i<7 ; i++)
			{
				robot->_M(i,i) += 0.1;
			}

			joint_task->computeTorques(joint_task_torques);
			posori_task_torques.setZero();

			int current_buffer_index = controller_counter % buffer_size;
			cumulated_error -= buffer_joint_task_error[current_buffer_index];
			buffer_joint_task_error[current_buffer_index] = (joint_task->_desired_position - joint_task->_current_position).norm();
			cumulated_error += buffer_joint_task_error[current_buffer_index];


			// cout << cumulated_error / buffer_size << endl;

			if(cumulated_error / buffer_size < 1e-2)
			{
				posori_task->reInitializeTask();
				state = WAIT_FOR_CAMERA_INFO;
				cout << "Wait For Camera Input" << endl << endl;
				joint_task->_kp = 0.0;
				joint_task->_kv = 5.0;
				joint_task->_ki = 0.0;
			}

		}

		else if(state == WAIT_FOR_CAMERA_INFO)
		{

			Matrix3d desired_rot_in_camera_frame = redis_client.getEigenMatrixJSON(DSIRED_ROT_IN_CAMERA_FRAME_KEY);
			Vector3d desired_pos_in_camera_frame = redis_client.getEigenMatrixJSON(DSIRED_POS_IN_CAMERA_FRAME_KEY); // + desired_rot_in_camera_frame * T_ee_camera.linear().transpose() * p_handBase_controlPoint;

			for(int i=3 ; i<6 ; i++)
			{
				posori_task->_Lambda(i,i) += 0.1;
			}

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			if(desired_pos_in_camera_frame.norm() > 1e-2)
			{
				desired_pos_in_camera_frame = desired_pos_in_camera_frame + desired_rot_in_camera_frame * T_ee_camera.linear().transpose() * p_handBase_controlPoint;

				Affine3d T_world_camera = Affine3d::Identity();
				robot->transform(T_world_camera, camera_link, T_ee_camera.translation());
				T_world_camera.linear() = T_world_camera.linear()*T_ee_camera.linear();
				Vector3d desired_pos_in_world_frame = T_world_camera * desired_pos_in_camera_frame;

				Matrix3d desired_rot_in_world_frame = T_world_camera.linear() * desired_rot_in_camera_frame * R_ee_hand.transpose();

				Matrix3d rot_world_hand = Matrix3d::Identity();
				robot->rotation(rot_world_hand, link_name);
				rot_world_hand = rot_world_hand * R_ee_hand;

				posori_task->_desired_position = desired_pos_in_world_frame + Vector3d(0,0,0.10); // - rot_world_hand*index_in_hand;
				posori_task->_desired_orientation = desired_rot_in_world_frame;
				redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, posori_task->_desired_position);
				state = MOVE_ABOVE_GRASP_POSE;

				joint_task->_desired_position(1) = -1.01516;

			}
		}

		else if(state == MOVE_ABOVE_GRASP_POSE)
		{
			// posori_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_POS_KEY);

			for(int i=3 ; i<6 ; i++)
			{
				posori_task->_Lambda(i,i) += 0.1;
			}

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			if((posori_task->_desired_position - posori_task->_current_position).norm() < 0.01)
			{
				posori_task->_desired_position = posori_task->_desired_position + Vector3d(0.0, 0.0, -0.10);
				state = MOVE_TO_GRASP_POSE;
				grasp_wait_counter = 2000;
			}
		}

		else if(state == MOVE_TO_GRASP_POSE)
		{
			// redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, hand_positions_temp);

			for(int i=3 ; i<6 ; i++)
			{
				posori_task->_Lambda(i,i) += 0.1;
			}

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			if((posori_task->_desired_position - posori_task->_current_position).norm() < 0.01  &&  grasp_wait_counter == 0)
			{
				// VectorXd hand_positions_temp = redis_client.getEigenMatrixJSON(SVH_RECEIVED_POSITION_KEY);
				// hand_positions_temp(3) += 5.0;
				VectorXd hand_positions_temp = redis_client.getEigenMatrixJSON(SVH_RECEIVED_POSITION_KEY) + hand_position_offset;
				redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, hand_positions_temp);
				state = GRASP;
				grasp_wait_counter = 2000;
			}

			grasp_wait_counter--;
		}

		else if(state == GRASP)
		{

			if(grasp_wait_counter < 1000)
			{
				VectorXd SVH_positions = redis_client.getEigenMatrixJSON(SVH_POSITIONS_KEY);
				VectorXd SVH_currents = redis_client.getEigenMatrixJSON(SVH_CURRENT_KEY);
				VectorXd SVH_desired_positions = redis_client.getEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY);

				double desired_current = -150.0;
				double Ka = 0.1;

				double index_distal_velocity_command = Ka * (SVH_currents(2) - desired_current);
				double ring_velocity_command = Ka * (SVH_currents(6) - desired_current);

				SVH_desired_positions(2) += 0.001 * index_distal_velocity_command;
				SVH_desired_positions(6) += 0.001 * ring_velocity_command;

				redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, SVH_desired_positions);
			}



			// posori_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_POS_KEY);

			for(int i=3 ; i<6 ; i++)
			{
				posori_task->_Lambda(i,i) += 0.1;
			}

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);


			if(grasp_wait_counter == 0)
			{
				posori_task->_desired_position = posori_task->_desired_position + Vector3d(0.0, 0.0, 0.25);
				state = LIFT;
			}

			grasp_wait_counter--;
		}

		else if(state == LIFT)
		{

			VectorXd SVH_positions = redis_client.getEigenMatrixJSON(SVH_POSITIONS_KEY);
			VectorXd SVH_currents = redis_client.getEigenMatrixJSON(SVH_CURRENT_KEY);
			VectorXd SVH_desired_positions = redis_client.getEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY);

			double desired_current = -150.0;
			double Ka = 0.1;

			double index_distal_velocity_command = Ka * (SVH_currents(2) - desired_current);
			double ring_velocity_command = Ka * (SVH_currents(6) - desired_current);

			SVH_desired_positions(2) += 0.001 * index_distal_velocity_command;
			SVH_desired_positions(6) += 0.001 * ring_velocity_command;

			redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, SVH_desired_positions);

			posori_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_POS_KEY);

			for(int i=3 ; i<6 ; i++)
			{
				posori_task->_Lambda(i,i) += 0.1;
			}

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			if(redis_client.get(GO_TO_INITIAL_KEY) == "1")
			{
				joint_task->reInitializeTask();
				cout << "Go To Initial Config" << endl;
				state = GO_TO_INIT_CONFIG;
				redis_client.set(GO_TO_INITIAL_KEY, "0");


				redis_client.setEigenMatrixJSON(DSIRED_POS_IN_CAMERA_FRAME_KEY, Vector3d::Zero());
				redis_client.setEigenMatrixJSON(DSIRED_ROT_IN_CAMERA_FRAME_KEY, Matrix3d::Identity());
				// redis_client.setEigenMatrixJSON(DSIRED_ROT_IN_CAMERA_FRAME_KEY, T_world_camera.linear().transpose() * init_config_orientation * R_ee_hand);
				// redis_client.setEigenMatrixJSON(DSIRED_ROT_IN_CAMERA_FRAME_KEY, init_config_orientation);

			}

		}
		
		// compute torques
		// for(int i=3 ; i<6 ; i++)
		// {
		// 	posori_task->_Lambda(i,i) += 0.1;
		// }

		// for(int i=4 ; i<7 ; i++)
		// {
		// 	robot->_M(i,i) += 0.1;
		// }

		// posori_task->computeTorques(posori_task_torques);
		// // ori_task->computeTorques(ori_task_torques);
		// joint_task->computeTorques(joint_task_torques);

		// if(state == GO_TO_INIT_CONFIG)
		// {
		// 	posori_task_torques.setZero(dof);
		// }

		// if(controller_counter % 100 == 0)
		// {
		// 	cout << posori_task_torques.transpose() << endl << endl;
		// }

		command_torques = posori_task_torques + joint_task_torques + coriolis;

		// send to redis
		// command_torques.setZero(dof);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		prev_time = current_time;
		controller_counter++;
	}

	command_torques.setZero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
