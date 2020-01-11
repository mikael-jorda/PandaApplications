#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "filters/ButterworthFilter.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"

#include <iostream>
#include <string>

#include "safe_ptr.h"

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
const vector<string> robot_names = {
	"PANDA1",
	"PANDA2",
};

const int n_robots = robot_names.size();

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


const string DSIRED_POS_IN_CAMERA_FRAME_KEY = "sai2::PandaApplication::controller::desired_pos_in_camera_frame";
const string DSIRED_ROT_IN_CAMERA_FRAME_KEY = "sai2::PandaApplication::controller::desired_rot_in_camera_frame";

const string GO_TO_INITIAL_KEY = "sai2::PandaApplication::controller::go_to_initial_pos";

const string SVH_HAND_COMMAND_POSITIONS_KEY = "sai2::SVHHand_Right::position_command";
const string SVH_RECEIVED_POSITION_KEY = "sai2::PandaApplication::controller::SVH_position_command";
const string SVH_CURRENT_KEY = "sai2::SVHHand_Right::current";
const string SVH_POSITIONS_KEY = "sai2::SVHHand_Right::position";


// function to update model at a slower rate
void updateModelThread(vector<sf::safe_ptr<Sai2Model::Sai2Model>> robots, 
		vector<sf::safe_ptr<Sai2Primitives::JointTask>> joint_tasks, 
		vector<sf::safe_ptr<Sai2Primitives::PosOriTask>> posori_tasks);

unsigned long long controller_counter = 0;

#define GO_TO_INIT_CONFIG          0
#define WAIT_FOR_CAMERA_INFO       1
#define MOVE_ABOVE_GRASP_POSE      2
#define MOVE_TO_GRASP_POSE         3
#define GRASP                      4
#define LIFT                       5
#define DEBUG                      10

int state = GO_TO_INIT_CONFIG;

const bool flag_simulation = false;
// const bool flag_simulation = true;

int main() {

	if(!flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEYS[0] = "sai2::FrankaPanda::Clyde::actuators::fgc";
		JOINT_ANGLES_KEYS[0]  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEYS[0] = "sai2::FrankaPanda::Clyde::sensors::dq";
		MASSMATRIX_KEYS[0] = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEYS[0] = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEYS[0] = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";		

		JOINT_TORQUES_COMMANDED_KEYS[1] = "sai2::FrankaPanda::Bonnie::actuators::fgc";
		JOINT_ANGLES_KEYS[1]  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::dq";
		MASSMATRIX_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_GRAVITY_KEYS[1] = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";	
	}


	// position of robots in world
	vector<Affine3d> robot_pose_in_world;
	Affine3d pose = Affine3d::Identity();
	robot_pose_in_world.push_back(pose);

	pose.translation() = Vector3d(0.273298,  1.08895,  0.0);
	pose.linear() = AngleAxisd(-1.39519, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	Affine3d T_World_Bonnie = robot_pose_in_world[1];

	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	vector<sf::safe_ptr<Sai2Model::Sai2Model>> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(sf::safe_ptr<Sai2Model::Sai2Model>(robot_files[i], false));
		robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
		robots[i]->updateModel();
	}

	// prepare task controllers
	vector<int> dof;
	vector<VectorXd> command_torques;
	vector<VectorXd> coriolis;
	vector<MatrixXd> N_prec;

	vector<sf::safe_ptr<Sai2Primitives::JointTask>> joint_tasks;
	vector<VectorXd> joint_task_torques;
	vector<sf::safe_ptr<Sai2Primitives::PosOriTask>> posori_tasks;
	vector<VectorXd> posori_task_torques;

	const vector<string> link_names =
	{
		"link7",
		"link7",
	};

	const vector<Vector3d> pos_in_link =
	{
		Vector3d(0.0, 0.0, 0.115),
		Vector3d(0.0504, -0.0976, 0.124),
	};

	vector<Matrix3d> rotation_in_link;
	Matrix3d temp_rot = Matrix3d::Identity();
	temp_rot = AngleAxisd(3.0/4.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();
	rotation_in_link.push_back(temp_rot);

	temp_rot << 1/sqrt(2),  1/sqrt(2),  0,
				1/sqrt(2), -1/sqrt(2),  0,
				   0     ,      0    , -1;  
	rotation_in_link.push_back(temp_rot);


	Affine3d T_eeB_camera = Affine3d::Identity();
	T_eeB_camera.translation() = pos_in_link[1];
	T_eeB_camera.linear() = rotation_in_link[1];

	Matrix3d R_eeC_hand = rotation_in_link[0];


	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		command_torques.push_back(VectorXd::Zero(dof[i]));
		coriolis.push_back(VectorXd::Zero(dof[i]));
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));

		// joint tasks
		joint_tasks.push_back(sf::safe_ptr<Sai2Primitives::JointTask>(robots[i].get_obj_ptr()));
		joint_task_torques.push_back(VectorXd::Zero(dof[i]));

		joint_tasks[i]->_kp = 50.0;
		joint_tasks[i]->_kv = 14.0;

		joint_tasks[i]->_otg->setMaxVelocity(M_PI/5);
		joint_tasks[i]->_otg->setMaxAcceleration(M_PI);
		joint_tasks[i]->_otg->setMaxJerk(3*M_PI);

		// end effector tasks
		posori_tasks.push_back(sf::safe_ptr<Sai2Primitives::PosOriTask>(robots[i].get_obj_ptr(), link_names[i], pos_in_link[i]));
		posori_task_torques.push_back(VectorXd::Zero(dof[i]));

		posori_tasks[i]->_kp_pos = 400.0;
		posori_tasks[i]->_kv_pos = 20.0;
		posori_tasks[i]->_kp_ori = 200.0;
		posori_tasks[i]->_kv_ori = 15.0;		

		posori_tasks[i]->_otg->setMaxLinearVelocity(0.1);
		posori_tasks[i]->_otg->setMaxLinearAcceleration(1.0);
		posori_tasks[i]->_otg->setMaxLinearJerk(3.0);

		posori_tasks[i]->_otg->setMaxAngularVelocity(M_PI/3);
		posori_tasks[i]->_otg->setMaxAngularAcceleration(M_PI);
		posori_tasks[i]->_otg->setMaxAngularJerk(3*M_PI);

	}

	vector<VectorXd> potential_field =
	{
		VectorXd::Zero(dof[0]),
		VectorXd::Zero(dof[1]),
	};

	// --------------------------------------
	// --------------------------------------
	// init configs
	// --------------------------------------
	// --------------------------------------
	redis_client.setEigenMatrixJSON(DSIRED_POS_IN_CAMERA_FRAME_KEY, Vector3d::Zero());
	redis_client.setEigenMatrixJSON(DSIRED_ROT_IN_CAMERA_FRAME_KEY, Matrix3d::Identity());
	redis_client.set(GO_TO_INITIAL_KEY, "0");


	vector<VectorXd> q_init =
	{
		VectorXd::Zero(dof[0]),
		VectorXd::Zero(dof[1]),
	};
	q_init[0] << -0.0560503,  -0.881276,   0.136037,   -1.72751,   -0.06887,    2.37994,  -0.667217;
	// q_init[1] << -0.8111,   1.1555,  2.42385, -1.21504,   1.7895,  2.43347, 0.620746;
	q_init[1] << -0.642836,   1.10865,   2.46559, -0.804433,    1.3181,    2.4675,   1.31391;

	VectorXd q_Bonnie_wait = VectorXd::Zero(dof[1]);
	q_Bonnie_wait << -0.968476,   1.06765,   2.67113,  -2.34798,    1.7432,   1.33183,   1.29608;
	
	VectorXd svh_init_config = VectorXd::Zero(9);
	VectorXd svh_pre_grasp_config = VectorXd::Zero(9);
	svh_init_config << 0, 0, 0, 0, 0, 0, 0, 0, 10;
	svh_pre_grasp_config << 0, 55, 5, 5, 5, 5, 5, 5, 10;
	redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, svh_init_config);

	VectorXd hand_position_offset = VectorXd::Zero(9);
	hand_position_offset << 0, 0, 0, 50, 0, 15, 10, 10, 0;

	int grasp_wait_counter = 0;

	// Vector3d hand_base_offset = Vector3d::Zero();
	const Vector3d direction_of_approach_hand_local = AngleAxisd(-M_PI/4, Vector3d(-1/sqrt(2), 1/sqrt(2), 0)).toRotationMatrix() * Vector3d::UnitZ();
	Vector3d direction_of_approach_hand_global = Vector3d::Zero();



	vector<double> buffer_joint_task_error;
	const int buffer_size = 50;
	double cumulated_error = 0;
	for(int i=0 ; i<buffer_size ; i++)
	{
		buffer_joint_task_error.push_back(100);
		cumulated_error += buffer_joint_task_error[i];
	}


	// --------------------------------------
	// --------------------------------------
	// --------------------------------------
	// --------------------------------------

	// start update_model thread
	thread model_update_thread(updateModelThread, robots, joint_tasks, posori_tasks);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	runloop = true;
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;
		dt = current_time - prev_time;

		// read robot state from redis and update robot model
		for(int i=0 ; i<n_robots ; i++)
		{
			robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
			robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);

			// robots[i]->updateModel();
			// robots[i]->coriolisForce(coriolis[i]);
		}

		if(state == DEBUG)
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				joint_task_torques[i].setZero();
				posori_task_torques[i].setZero();
			}

		}

		else if(state == GO_TO_INIT_CONFIG)
		{
			redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, svh_init_config);

			for(int i=0 ; i<n_robots ; i++)
			{
				joint_tasks[i]->_desired_position = q_init[i];

				joint_tasks[i]->_kp = 300.0;
				joint_tasks[i]->_kv = 15.0;
				joint_tasks[i]->_ki = 35.0;

				joint_tasks[i]->computeTorques(joint_task_torques[i]);
				posori_task_torques[i].setZero();
			}


			int current_buffer_index = controller_counter % buffer_size;
			cumulated_error -= buffer_joint_task_error[current_buffer_index];
			buffer_joint_task_error[current_buffer_index] = (joint_tasks[1]->_desired_position - joint_tasks[1]->_current_position).norm() + (joint_tasks[0]->_desired_position - joint_tasks[0]->_current_position).norm();
			cumulated_error += buffer_joint_task_error[current_buffer_index];

			// cout << cumulated_error / buffer_size << endl;

			if(cumulated_error / buffer_size < 3e-2)
			{

				posori_tasks[0]->reInitializeTask();
				state = WAIT_FOR_CAMERA_INFO;
				cout << "Wait For Camera Input" << endl << endl;
				joint_tasks[0]->_kp = 0.0;
				joint_tasks[0]->_kv = 5.0;
				joint_tasks[0]->_ki = 0.0;
				joint_tasks[1]->_ki = 0.0;
			}

		}

		else if(state == WAIT_FOR_CAMERA_INFO)
		{

			Matrix3d desired_rot_in_camera_frame = redis_client.getEigenMatrixJSON(DSIRED_ROT_IN_CAMERA_FRAME_KEY);
			Vector3d desired_pos_in_camera_frame = redis_client.getEigenMatrixJSON(DSIRED_POS_IN_CAMERA_FRAME_KEY); // + desired_rot_in_camera_frame * T_ee_camera.linear().transpose() * p_handBase_controlPoint;

			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);


			if(desired_pos_in_camera_frame.norm() > 1e-2)
			{
				Affine3d T_world_camera = Affine3d::Identity();
				Affine3d T_Bonnie_eeB = Affine3d::Identity();
				robots[1]->transform(T_Bonnie_eeB, link_names[1]);

				T_world_camera = T_World_Bonnie * T_Bonnie_eeB * T_eeB_camera;

				Vector3d desired_pos_in_world_frame = T_world_camera * desired_pos_in_camera_frame;
				Matrix3d desired_rot_in_world_frame = T_world_camera.linear() * desired_rot_in_camera_frame * R_eeC_hand.transpose();


				posori_tasks[0]->_desired_orientation = desired_rot_in_world_frame;

				
				// hand_base_offset(0) = desired_rot_in_world_frame(0,2);
				// hand_base_offset(1) = desired_rot_in_world_frame(1,2);
				// hand_base_offset.normalize();
				// hand_base_offset = - 0.05 * hand_base_offset;
				// hand_base_offset(2) = 0.03;
				// posori_tasks[0]->_desired_position = desired_pos_in_world_frame + Vector3d(-0.05, 0.0, 0.05);
				// posori_tasks[0]->_desired_position = desired_pos_in_world_frame + Vector3d(-0.0, 0.0, 0.10);
				direction_of_approach_hand_global = desired_rot_in_world_frame * direction_of_approach_hand_local;
				posori_tasks[0]->_desired_position = desired_pos_in_world_frame - 0.05 * direction_of_approach_hand_global;
				state = MOVE_ABOVE_GRASP_POSE;

				joint_tasks[1]->_desired_position = q_Bonnie_wait;

				// VectorXd hand_positions_temp = redis_client.getEigenMatrixJSON(SVH_RECEIVED_POSITION_KEY);// - hand_position_offset;
				// redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, hand_positions_temp);
				redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, svh_pre_grasp_config);

			}
		}

		else if(state == MOVE_ABOVE_GRASP_POSE)
		{

			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			if((posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() < 0.013)
			{
				posori_tasks[0]->_desired_position = posori_tasks[0]->_desired_position + 0.05 * direction_of_approach_hand_global;
				state = MOVE_TO_GRASP_POSE;
				grasp_wait_counter = 1000;
			}
		}

		else if(state == MOVE_TO_GRASP_POSE)
		{
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);


			if((posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() < 0.013  &&  grasp_wait_counter == 0)
			{
				// VectorXd hand_positions_temp = redis_client.getEigenMatrixJSON(SVH_RECEIVED_POSITION_KEY);
				// hand_positions_temp(3) += 5.0;
				// 
				Vector3d pushing_amount = direction_of_approach_hand_global;
				pushing_amount(2) = 0;
				pushing_amount.normalize();

				posori_tasks[0]->_desired_position = posori_tasks[0]->_desired_position + 0.01 * pushing_amount;
				
				VectorXd hand_positions_temp = redis_client.getEigenMatrixJSON(SVH_RECEIVED_POSITION_KEY) + hand_position_offset;
				redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, hand_positions_temp);
				state = GRASP;
				grasp_wait_counter = 1000;
			}

			grasp_wait_counter--;
		}

		else if(state == GRASP)
		{

			// if(grasp_wait_counter < 1000)
			// {
			// 	VectorXd SVH_positions = redis_client.getEigenMatrixJSON(SVH_POSITIONS_KEY);
			// 	VectorXd SVH_currents = redis_client.getEigenMatrixJSON(SVH_CURRENT_KEY);
			// 	VectorXd SVH_desired_positions = redis_client.getEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY);

			// 	double desired_current = -150.0;
			// 	double Ka = 0.1;

			// 	double index_distal_velocity_command = Ka * (SVH_currents(2) - desired_current);
			// 	double middle_distal_velocity_command = Ka * (SVH_currents(4) - desired_current);
			// 	double ring_velocity_command = Ka * (SVH_currents(6) - desired_current);
			// 	double pinky_velocity_command = Ka * (SVH_currents(7) - desired_current);

			// 	SVH_desired_positions(2) += 0.001 * index_distal_velocity_command;
			// 	SVH_desired_positions(4) += 0.001 * middle_distal_velocity_command;
			// 	SVH_desired_positions(6) += 0.001 * ring_velocity_command;
			// 	SVH_desired_positions(7) += 0.001 * pinky_velocity_command;

			// 	redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, SVH_desired_positions);
			// }


			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);
			
			if(grasp_wait_counter == 0)
			{
				posori_tasks[0]->_desired_position = posori_tasks[0]->_desired_position + Vector3d(0.0, 0.0, 0.07);
				state = LIFT;
			}

			grasp_wait_counter--;
		}

		else if(state == LIFT)
		{

			// VectorXd SVH_positions = redis_client.getEigenMatrixJSON(SVH_POSITIONS_KEY);
			// VectorXd SVH_currents = redis_client.getEigenMatrixJSON(SVH_CURRENT_KEY);
			// VectorXd SVH_desired_positions = redis_client.getEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY);

			// double desired_current = -150.0;
			// double Ka = 0.1;

			// double index_distal_velocity_command = Ka * (SVH_currents(2) - desired_current);
			// double middle_distal_velocity_command = Ka * (SVH_currents(4) - desired_current);
			// double ring_velocity_command = Ka * (SVH_currents(6) - desired_current);
			// double pinky_velocity_command = Ka * (SVH_currents(7) - desired_current);

			// SVH_desired_positions(2) += 0.001 * index_distal_velocity_command;
			// SVH_desired_positions(4) += 0.001 * middle_distal_velocity_command;
			// SVH_desired_positions(6) += 0.001 * ring_velocity_command;
			// SVH_desired_positions(7) += 0.001 * pinky_velocity_command;

			// redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, SVH_desired_positions);

			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			if(redis_client.get(GO_TO_INITIAL_KEY) == "1")
			{
				joint_tasks[0]->reInitializeTask();
				joint_tasks[1]->reInitializeTask();
				cout << "Go To Initial Config" << endl;
				state = GO_TO_INIT_CONFIG;
				redis_client.set(GO_TO_INITIAL_KEY, "0");


				redis_client.setEigenMatrixJSON(DSIRED_POS_IN_CAMERA_FRAME_KEY, Vector3d::Zero());
				redis_client.setEigenMatrixJSON(DSIRED_ROT_IN_CAMERA_FRAME_KEY, Matrix3d::Identity());

			}

		}
		

		// compute potential field to send joint 1 away from zero
		double pot_field_init_value = 0.65;
		double pot_field_max_torque = 100.0;

		double pot_field_force = 0.0;

		double q1 = robots[0]->_q(1);

		if(q1 > -pot_field_init_value && q1 < 0)
		{
			pot_field_force = - pot_field_max_torque / 2 * (1 - cos((q1 + pot_field_init_value)  /pot_field_init_value*M_PI));
		}
		else if(q1 < pot_field_init_value && q1 > 0)
		{
			pot_field_force = pot_field_max_torque / 2 * (1 - cos((q1 - pot_field_init_value)  /pot_field_init_value*M_PI));
		}
		else
		{
			pot_field_force = 0.0;
		}

		// if(state >= MOVE_TO_GRASP_POSE)
		// {
		// 	pot_field_force = 0;
		// }

		potential_field[0](1) = pot_field_force;

		if(controller_counter % 100 == 0)
		{
		// 	cout << q1 << endl;
			// cout << (posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() << endl;
			// cout << pot_field_force << endl;
			// cout << endl;
		}

		for(int i = 0 ; i<n_robots ; i++)
		{

			command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i] + potential_field[i];

			// command_torques.setZero(dof);
			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
		}

		prev_time = current_time;
		controller_counter++;
	}

	for(int i=0 ; i<n_robots ; i++)
	{
		command_torques[i].setZero();
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    // for(int i=0 ; i<n_robots ; i++)
    // {
    // 	delete robots[i];
    // 	delete posori_tasks[i];
    // 	delete joint_tasks[i];
    // }

	return 0;
}

void updateModelThread(vector<sf::safe_ptr<Sai2Model::Sai2Model>> robots, 
		vector<sf::safe_ptr<Sai2Primitives::JointTask>> joint_tasks, 
		vector<sf::safe_ptr<Sai2Primitives::PosOriTask>> posori_tasks)
{

	// prepare task controllers
	vector<int> dof;
	vector<MatrixXd> N_prec;

	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));
	}

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while(runloop)
	{
		timer.waitForNextLoop();
		for(int i=0 ; i<n_robots ; i++)
		{
			robots[i]->updateModel();
		}

		if(state == DEBUG)
		{

		}

		else if(state == GO_TO_INIT_CONFIG)
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				joint_tasks[i]->updateTaskModel(N_prec[i]);

				for(int j=4 ; j<7 ; j++)
				{
					robots[i]->_M(j,j) += 0.1;
				}
			}
		}

		else
		{
			N_prec[0].setIdentity();
			posori_tasks[0]->updateTaskModel(N_prec[0]);
			N_prec[0] = posori_tasks[0]->_N;
			joint_tasks[0]->updateTaskModel(N_prec[0]);

			N_prec[1].setIdentity();
			joint_tasks[1]->updateTaskModel(N_prec[1]);

			for(int j=4 ; j<7 ; j++)
			{
				robots[1]->_M(j,j) += 0.1;
			}
		
			for(int j=3 ; j<6 ; j++)
			{
				posori_tasks[0]->_Lambda(j,j) += 0.1;
			}
		}

	}
}