#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "filters/ButterworthFilter.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"

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

// camera stuff
const string CAMERA_FINISHED_KEY = "sai2::PandaApplication::camera_finished";
const string DESIRED_FINGERTIP_POS_IN_CAMERA_FRAME_KEY = "sai2::PandaApplication::desired_fingertip_pos_in_camera_frame";
const string DESIRED_FINGERTIP_POS_IN_WORLD_FRAME_KEY = "sai2::PandaApplication::desired_fingertip_pos_in_world_frame";

// const string DESIRED_FINGERTIP_POS_IN_CAMERA_FRAME_KEY = "sai2::PandaApplication::controller::desired_pos_in_camera_frame";
// const string DSIRED_ROT_IN_CAMERA_FRAME_KEY = "sai2::PandaApplication::controller::desired_rot_in_camera_frame";

const string GO_TO_INITIAL_KEY = "sai2::PandaApplication::controller::go_to_initial_pos";


// ik keys
const string ALLEGRO_DESIRED_JOINT_POSITIONS_FROM_IK = "sai2::unigraspAllegro::desired_finger_configuration_from_IK";
const string DESIRED_PALM_POSITION_FROM_IK_KEY = "sai2::unigraspAllegro::desired_palm_position_from_IK";
const string DESIRED_PALM_ORIENTATION_FROM_IK_KEY = "sai2::unigraspAllegro::desired_palm_orientation_from_IK";
const string IK_FINISHED_KEY = "sai2::unigraspAllegro::ik_finished";


// allegro driver keys
const string ALLEGRO_JOINT_POSITIONS = "sai2::allegroHand::sensors::joint_positions";
const string ALLEGRO_COMMANDED_JOINT_POSITIONS = "sai2::allegroHand::controller::joint_positions_commanded";
const string ALLEGRO_COMMANDED_JOINT_TORQUES = "sai2::allegroHand::controller::joint_torques_commanded";
const string ALLEGRO_PALM_ORIENTATION_KEY = "sai2::allegroHand::controller::palm_orientation";
const string ALLEGRO_CONTROL_MODE = "sai2::allegroHand::controller::control_mode";


// function to update model at a slower rate
void updateModelThread(vector<shared_ptr<Sai2Model::Sai2Model>> robots, 
		vector<shared_ptr<Sai2Primitives::JointTask>> joint_tasks, 
		vector<shared_ptr<Sai2Primitives::PosOriTask>> posori_tasks);

unsigned long long controller_counter = 0;

#define GO_TO_INIT_CONFIG          0
#define WAIT_FOR_CAMERA_AND_IK     1
#define MOVE_ABOVE_GRASP_POSE      2
#define MOVE_TO_GRASP_POSE         3
#define GRASP                      4
#define LIFT                       5
#define DEBUG                      10

int state = GO_TO_INIT_CONFIG;

bool flagcout = true;

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
	vector<shared_ptr<Sai2Model::Sai2Model>> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(make_shared<Sai2Model::Sai2Model>(robot_files[i], false));
		robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
		robots[i]->updateModel();
	}

	// prepare task controllers
	vector<int> dof;
	vector<VectorXd> command_torques;
	vector<VectorXd> coriolis;
	vector<MatrixXd> N_prec;

	vector<shared_ptr<Sai2Primitives::JointTask>> joint_tasks;
	vector<VectorXd> joint_task_torques;
	vector<shared_ptr<Sai2Primitives::PosOriTask>> posori_tasks;
	vector<VectorXd> posori_task_torques;

	const vector<string> link_names =
	{
		"link7",
		"link7",
	};

	const vector<Vector3d> pos_in_link =
	{
		// Vector3d(0.131, 0, 0.2032),
		Vector3d(0.05, 0.0, 0.15),
		// Vector3d(0.0504, -0.0976, 0.124),
		Vector3d(0.0504, -0.0976, 0.05),
	};

	// vector<Matrix3d> rotation_in_link;
	// Matrix3d temp_rot = Matrix3d::Identity();
	// temp_rot = AngleAxisd(3.0/4.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();
	// rotation_in_link.push_back(temp_rot);

	// temp_rot << 1/sqrt(2),  1/sqrt(2),  0,
	// 			1/sqrt(2), -1/sqrt(2),  0,
	// 			   0     ,      0    , -1;  
	// rotation_in_link.push_back(temp_rot);


	Affine3d T_eeB_camera = Affine3d::Identity();
	T_eeB_camera.translation() = pos_in_link[1];
	T_eeB_camera.linear() << 1/sqrt(2),  1/sqrt(2),  0,
							 1/sqrt(2), -1/sqrt(2),  0,
							    0     ,      0    , -1;  

	Matrix3d R_eeC_hand = Matrix3d::Identity();
	R_eeC_hand <<  0, 0, 1,
				   0,-1, 0,
				   1, 0, 0;


	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		command_torques.push_back(VectorXd::Zero(dof[i]));
		coriolis.push_back(VectorXd::Zero(dof[i]));
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));

		// joint tasks
		joint_tasks.push_back(make_shared<Sai2Primitives::JointTask>(robots[i].get()));
		joint_task_torques.push_back(VectorXd::Zero(dof[i]));

		joint_tasks[i]->_kp = 50.0;
		joint_tasks[i]->_kv = 14.0;

		joint_tasks[i]->_otg->setMaxVelocity(M_PI/5);
		joint_tasks[i]->_otg->setMaxAcceleration(M_PI);
		joint_tasks[i]->_otg->setMaxJerk(3*M_PI);

		// end effector tasks
		posori_tasks.push_back(make_shared<Sai2Primitives::PosOriTask>(robots[i].get(), link_names[i], pos_in_link[i]));
		posori_task_torques.push_back(VectorXd::Zero(dof[i]));

		posori_tasks[i]->_kp_pos = 400.0;
		posori_tasks[i]->_kv_pos = 20.0;
		posori_tasks[i]->_kp_ori = 200.0;
		posori_tasks[i]->_kv_ori = 15.0;		

		posori_tasks[i]->_otg->setMaxLinearVelocity(0.15);
		posori_tasks[i]->_otg->setMaxLinearAcceleration(0.3);
		posori_tasks[i]->_otg->setMaxLinearJerk(0.5);

		posori_tasks[i]->_otg->setMaxAngularVelocity(M_PI/3);
		posori_tasks[i]->_otg->setMaxAngularAcceleration(M_PI/6);
		posori_tasks[i]->_otg->setMaxAngularJerk(M_PI/2);

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
	redis_client.setEigenMatrixJSON(DESIRED_FINGERTIP_POS_IN_CAMERA_FRAME_KEY, VectorXd::Zero(9));
	redis_client.setEigenMatrixJSON(DESIRED_FINGERTIP_POS_IN_WORLD_FRAME_KEY, VectorXd::Zero(9));
	redis_client.set(GO_TO_INITIAL_KEY, "0");
	redis_client.set(IK_FINISHED_KEY, "0");


	vector<VectorXd> q_init =
	{
		VectorXd::Zero(dof[0]),
		VectorXd::Zero(dof[1]),
	};
	q_init[0] << 0.847144, -0.0343682, -0.637469, -1.85414, -0.103955, 1.87454, 0.496223;
	// q_init[1] << 2.28843,-1.03064,-1.3612,-1.8834,2.14458,1.59748,0.491425;
	q_init[1] << 2.00311,-1.05719,-1.37171,-1.91844,2.09774,1.51831,0.532613;
	// q_init[1] << 2.05321,-1.11204,-1.25244,-2.27785,2.09808,1.39183,0.51338;

	VectorXd q_Bonnie_wait = VectorXd::Zero(dof[1]);
	q_Bonnie_wait << 2.31775,0.284248,-1.0975,-1.74569,2.1436,1.59819,0.49137;
	

	VectorXd allegro_init_config = VectorXd::Zero(16);
	VectorXd allegro_grasp_config = VectorXd::Zero(16);
	VectorXd allegro_commanded_positons = VectorXd::Zero(16);

	allegro_init_config << 0.0671049,0.788217,0.863665,0.923758,0.0626668,0.90281,0.744723,0.809697,0.0387895,0.864198,0.942931,0.811295,-1.22342,0.117256,0.189332,0.768245;

	allegro_commanded_positons = allegro_init_config;
	redis_client.setEigenMatrixJSON(ALLEGRO_COMMANDED_JOINT_POSITIONS, allegro_commanded_positons);

	Matrix3d R_palm = Matrix3d::Identity();
	robots[0]->rotation(R_palm, link_names[0]);
	R_palm = R_palm * R_eeC_hand;
	redis_client.setEigenMatrixJSON(ALLEGRO_PALM_ORIENTATION_KEY, R_palm);
	redis_client.set(ALLEGRO_CONTROL_MODE, "p");

	VectorXd allegro_grasp_position_offset = VectorXd::Zero(16);
	allegro_grasp_position_offset << 0, 0.07, 0, 0, 0, 0.07, 0, 0, 0, 0, 0, 0, 0, -0.07, 0, 0;



	// VectorXd svh_init_config = VectorXd::Zero(9);
	// VectorXd svh_pre_grasp_config = VectorXd::Zero(9);
	// svh_init_config << 0, 0, 0, 0, 0, 0, 0, 0, 10;
	// svh_pre_grasp_config << 0, 55, 5, 5, 5, 5, 5, 5, 10;
	// redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, svh_init_config);

	// VectorXd hand_position_offset = VectorXd::Zero(9);
	// hand_position_offset << 0, 0, 0, 50, 0, 15, 10, 10, 0;

	int grasp_wait_counter = 0;

	// Vector3d hand_base_offset = Vector3d::Zero();
	// const Vector3d direction_of_approach_hand_local = AngleAxisd(-M_PI/4, Vector3d(-1/sqrt(2), 1/sqrt(2), 0)).toRotationMatrix() * Vector3d::UnitZ();
	const Vector3d direction_of_approach_hand_local = AngleAxisd(M_PI/6, Vector3d::UnitY()).toRotationMatrix() * Vector3d::UnitZ();
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

	VectorXd desired_fingertip_pos_in_camera_frame = VectorXd::Zero(9);
	VectorXd desired_fingertip_pos_in_world_frame = VectorXd::Zero(9);
	int camera_finished = 0;
	Vector3d desired_palm_position_from_ik = Vector3d::Zero();
	Matrix3d desired_palm_orientation_from_ik = Matrix3d::Identity();
	// prepare redis things to read and write
	for(int i=0 ; i<n_robots ; i++)
	{

		redis_client.addEigenToRead(JOINT_ANGLES_KEYS[i], robots[i]->_q);
		redis_client.addEigenToRead(JOINT_VELOCITIES_KEYS[i], robots[i]->_dq);

		if(!flag_simulation)
		{
			redis_client.addEigenToRead(MASSMATRIX_KEYS[i], robots[i]->_M);
			redis_client.addEigenToRead(CORIOLIS_KEYS[i], coriolis[i]);
		}
	}
	redis_client.set(CAMERA_FINISHED_KEY, "0");

	redis_client.addIntToRead(CAMERA_FINISHED_KEY, camera_finished);
	redis_client.addEigenToRead(DESIRED_FINGERTIP_POS_IN_CAMERA_FRAME_KEY, desired_fingertip_pos_in_camera_frame);

	redis_client.addEigenToRead(ALLEGRO_DESIRED_JOINT_POSITIONS_FROM_IK, allegro_grasp_config);

	redis_client.addEigenToRead(DESIRED_PALM_POSITION_FROM_IK_KEY, desired_palm_position_from_ik);
	redis_client.addEigenToRead(DESIRED_PALM_ORIENTATION_FROM_IK_KEY, desired_palm_orientation_from_ik);
	
	// write
	redis_client.addEigenToWrite(ALLEGRO_PALM_ORIENTATION_KEY, R_palm);

	redis_client.addEigenToWrite(ALLEGRO_COMMANDED_JOINT_POSITIONS, allegro_commanded_positons);
	redis_client.addEigenToWrite(DESIRED_FINGERTIP_POS_IN_WORLD_FRAME_KEY , desired_fingertip_pos_in_world_frame);

	for(int i=0 ; i<n_robots ; i++)
	{
		redis_client.addEigenToWrite(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
	}

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

		// read redis
		redis_client.readAllSetupValues();

		// for(int i=0 ; i<n_robots ; i++)
		// {
		// 	if(flag_simulation)
		// 	{
		// 		robots[i]->updateModel();
		// 		// robots[i]->coriolisForce(coriolis[i]);
		// 	}
		// 	else
		// 	{
		// 		robots[i]->updateInverseInertia();
		// 	}
		// }

		// if(state == GO_TO_INIT_CONFIG)
		// {
		// 	for(int i=0 ; i<n_robots ; i++)
		// 	{
		// 		N_prec[i].setIdentity();
		// 		joint_tasks[i]->updateTaskModel(N_prec[i]);
		// 	}
		// }

		// else
		// {
		// 	N_prec[0].setIdentity();
		// 	posori_tasks[0]->updateTaskModel(N_prec[0]);
		// 	N_prec[0] = posori_tasks[0]->_N;
		// 	joint_tasks[0]->updateTaskModel(N_prec[0]);

		// 	N_prec[1].setIdentity();
		// 	joint_tasks[1]->updateTaskModel(N_prec[1]);
		// }

		// read robot state from redis and update robot model
		for(int i=0 ; i<n_robots ; i++)
		{
			// robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
			// robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);

			robots[i]->updateKinematics();
			// robots[i]->coriolisForce(coriolis[i]);
		}

		// send palm orientation to allegro driver for grav comp
		robots[0]->rotation(R_palm, link_names[0]);
		R_palm = R_palm * R_eeC_hand;
		// redis_client.setEigenMatrixJSON(ALLEGRO_PALM_ORIENTATION_KEY, R_palm);

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
			allegro_commanded_positons = allegro_init_config;

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

			if(cumulated_error / buffer_size < 0.1)
			{

				posori_tasks[0]->reInitializeTask();
				state = WAIT_FOR_CAMERA_AND_IK;
				cout << "Wait For Camera Input" << endl << endl;
				joint_tasks[0]->_kp = 0.0;
				joint_tasks[0]->_kv = 5.0;
				joint_tasks[0]->_ki = 0.0;
				joint_tasks[1]->_ki = 0.0;
			}

		}

		else if(state == WAIT_FOR_CAMERA_AND_IK)
		{
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			// if(desired_fingertip_pos_in_camera_frame.norm() > 1e-2)
			if(desired_fingertip_pos_in_camera_frame.norm() > 1e-2 && camera_finished == 1)
			{
				Affine3d T_world_camera = Affine3d::Identity();
				Affine3d T_Bonnie_eeB = Affine3d::Identity();
				robots[1]->transform(T_Bonnie_eeB, link_names[1]);
				T_world_camera = T_World_Bonnie * T_Bonnie_eeB * T_eeB_camera;

				for(int i=0 ; i<3 ; i++)
				{
					desired_fingertip_pos_in_world_frame.segment<3>(3*i) = T_world_camera * desired_fingertip_pos_in_camera_frame.segment<3>(3*i);
				}

				if(flagcout)
				{
					cout << "Waiting for IK input" << endl;
					flagcout = false;
				}
			}
			if(redis_client.get(IK_FINISHED_KEY) == "1")
			{
				flagcout = true;
				// allegro_commanded_positons = allegro_grasp_config - allegro_grasp_position_offset;
				allegro_commanded_positons = allegro_grasp_config;

				posori_tasks[0]->_desired_orientation = desired_palm_orientation_from_ik;
				posori_tasks[0]->_desired_position = desired_palm_position_from_ik;

				direction_of_approach_hand_global = posori_tasks[0]->_desired_orientation * direction_of_approach_hand_local;
				// posori_tasks[0]->_desired_position -= 0.05 * direction_of_approach_hand_global;
				posori_tasks[0]->_desired_position += 0.02 * Vector3d::UnitZ();

				joint_tasks[1]->_desired_position = q_Bonnie_wait;

				redis_client.set(IK_FINISHED_KEY, "0");

				state = MOVE_ABOVE_GRASP_POSE;
			}



			// if(desired_palm_position_from_ik.norm() > 1e-2 && camera_finished == 1)
			// {
			// 	Affine3d T_world_camera = Affine3d::Identity();
			// 	Affine3d T_Bonnie_eeB = Affine3d::Identity();
			// 	robots[1]->transform(T_Bonnie_eeB, link_names[1]);
			// 	T_world_camera = T_World_Bonnie * T_Bonnie_eeB * T_eeB_camera;

			// 	allegro_commanded_positons = allegro_grasp_config;

			// 	posori_tasks[0]->_desired_orientation = T_world_camera.linear() * desired_palm_orientation_from_ik * R_eeC_hand.transpose();
			// 	// posori_tasks[0]->_desired_orientation = T_world_camera.linear() * desired_palm_orientation_from_ik;
			// 	posori_tasks[0]->_desired_position = T_world_camera * desired_palm_position_from_ik;

			// 	direction_of_approach_hand_global = posori_tasks[0]->_desired_orientation * direction_of_approach_hand_local;
			// 	// posori_tasks[0]->_desired_position -= 0.05 * direction_of_approach_hand_global;
			// 	posori_tasks[0]->_desired_position += 0.02 * Vector3d::UnitZ();

			// 	joint_tasks[1]->_desired_position = q_Bonnie_wait;

			// 	state = MOVE_ABOVE_GRASP_POSE;
			// }


			// if(desired_pos_in_camera_frame.norm() > 1e-2)
			// {
			// 	Affine3d T_world_camera = Affine3d::Identity();
			// 	Affine3d T_Bonnie_eeB = Affine3d::Identity();
			// 	robots[1]->transform(T_Bonnie_eeB, link_names[1]);

			// 	T_world_camera = T_World_Bonnie * T_Bonnie_eeB * T_eeB_camera;

			// 	Vector3d desired_pos_in_world_frame = T_world_camera * desired_pos_in_camera_frame;
			// 	Matrix3d desired_rot_in_world_frame = T_world_camera.linear() * desired_rot_in_camera_frame * R_eeC_hand.transpose();


			// 	posori_tasks[0]->_desired_orientation = desired_rot_in_world_frame;

				
			// 	// hand_base_offset(0) = desired_rot_in_world_frame(0,2);
			// 	// hand_base_offset(1) = desired_rot_in_world_frame(1,2);
			// 	// hand_base_offset.normalize();
			// 	// hand_base_offset = - 0.05 * hand_base_offset;
			// 	// hand_base_offset(2) = 0.03;
			// 	// posori_tasks[0]->_desired_position = desired_pos_in_world_frame + Vector3d(-0.05, 0.0, 0.05);
			// 	// posori_tasks[0]->_desired_position = desired_pos_in_world_frame + Vector3d(-0.0, 0.0, 0.10);
			// 	direction_of_approach_hand_global = desired_rot_in_world_frame * direction_of_approach_hand_local;
			// 	posori_tasks[0]->_desired_position = desired_pos_in_world_frame - 0.05 * direction_of_approach_hand_global;
			// 	state = MOVE_ABOVE_GRASP_POSE;

			// 	joint_tasks[1]->_desired_position = q_Bonnie_wait;

			// 	// VectorXd hand_positions_temp = redis_client.getEigenMatrixJSON(SVH_RECEIVED_POSITION_KEY);// - hand_position_offset;
			// 	// redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, hand_positions_temp);
			// 	redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, svh_pre_grasp_config);

			// }
		}

		else if(state == MOVE_ABOVE_GRASP_POSE)
		{
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			// if((posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() < 0.013)
			// {
			// 	posori_tasks[0]->_desired_position += 0.05 * direction_of_approach_hand_global;
			// 	state = MOVE_TO_GRASP_POSE;
			// 	cout << "move to grasp pose" << endl;
			// 	grasp_wait_counter = 1000;
			// }
		}

		else if(state == MOVE_TO_GRASP_POSE)
		{
			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			// cout << (posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() << endl;

			if((posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() < 0.013  &&  grasp_wait_counter <= 0)
			{
				// VectorXd hand_positions_temp = redis_client.getEigenMatrixJSON(SVH_RECEIVED_POSITION_KEY);
				// hand_positions_temp(3) += 5.0;
				// 
				// Vector3d pushing_amount = direction_of_approach_hand_global;
				// pushing_amount(2) = 0;
				// pushing_amount.normalize();

				// posori_tasks[0]->_desired_position = posori_tasks[0]->_desired_position + 0.01 * pushing_amount;
				
				allegro_commanded_positons = allegro_grasp_config + allegro_grasp_position_offset;


				// VectorXd hand_positions_temp = redis_client.getEigenMatrixJSON(SVH_RECEIVED_POSITION_KEY) + hand_position_offset;
				// redis_client.setEigenMatrixJSON(SVH_HAND_COMMAND_POSITIONS_KEY, hand_positions_temp);
				cout << "grasp" << endl;
				state = GRASP;
				grasp_wait_counter = 1000;
			}

			grasp_wait_counter--;
		}

		else if(state == GRASP)
		{

			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);
			
			if(grasp_wait_counter == 0)
			{
				posori_tasks[0]->_desired_position += Vector3d(0.0, 0.0, 0.07);
				cout << "lift" << endl;
				state = LIFT;
			}

			grasp_wait_counter--;
		}

		else if(state == LIFT)
		{

			posori_tasks[0]->computeTorques(posori_task_torques[0]);
			joint_tasks[0]->computeTorques(joint_task_torques[0]);

			joint_tasks[1]->computeTorques(joint_task_torques[1]);

			if(redis_client.get(GO_TO_INITIAL_KEY) == "1")
			{
				joint_tasks[0]->reInitializeTask();
				joint_tasks[1]->reInitializeTask();
				cout << "Go To Initial Config" << endl;
				redis_client.set(GO_TO_INITIAL_KEY, "0");
				redis_client.set(CAMERA_FINISHED_KEY, "0");
				redis_client.set(IK_FINISHED_KEY, "0");
				redis_client.setEigenMatrixJSON(DESIRED_FINGERTIP_POS_IN_CAMERA_FRAME_KEY, VectorXd::Zero(9));
				state = GO_TO_INIT_CONFIG;

			}

		}
		

		// // compute potential field to send joint 1 away from zero
		// double pot_field_init_value = 0.65;
		// double pot_field_max_torque = 100.0;

		// double pot_field_force = 0.0;

		// double q1 = robots[0]->_q(1);

		// if(q1 > -pot_field_init_value && q1 < 0)
		// {
		// 	pot_field_force = - pot_field_max_torque / 2 * (1 - cos((q1 + pot_field_init_value)  /pot_field_init_value*M_PI));
		// }
		// else if(q1 < pot_field_init_value && q1 > 0)
		// {
		// 	pot_field_force = pot_field_max_torque / 2 * (1 - cos((q1 - pot_field_init_value)  /pot_field_init_value*M_PI));
		// }
		// else
		// {
		// 	pot_field_force = 0.0;
		// }

		// if(state >= MOVE_TO_GRASP_POSE)
		// {
		// 	pot_field_force = 0;
		// }

		// potential_field[0](1) = pot_field_force;

		if(controller_counter % 100 == 0)
		{
		// 	cout << q1 << endl;
			// cout << (posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() << endl;
			// cout << pot_field_force << endl;
			// cout << endl;
		}

		for(int i = 0 ; i<n_robots ; i++)
		{

			// command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i] + potential_field[i];
			command_torques[i] = posori_task_torques[i] + joint_task_torques[i];

			if(isnan(command_torques[i](0)))
			{
				cout << "main nan" << endl;
			}
			// command_torques.setZero(dof);
			// redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
		}

		redis_client.writeAllSetupValues();

		prev_time = current_time;
		controller_counter++;
	}

	model_update_thread.join();

	for(int i=0 ; i<n_robots ; i++)
	{
		command_torques[i].setZero();
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
	}

	redis_client.set(ALLEGRO_CONTROL_MODE, "t");
	redis_client.setEigenMatrixJSON(ALLEGRO_COMMANDED_JOINT_TORQUES, VectorXd::Zero(16));

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

void updateModelThread(vector<shared_ptr<Sai2Model::Sai2Model>> robots, 
		vector<shared_ptr<Sai2Primitives::JointTask>> joint_tasks, 
		vector<shared_ptr<Sai2Primitives::PosOriTask>> posori_tasks)
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
			if(flag_simulation)
			{
				robots[i]->updateModel();
				// robots[i]->coriolisForce(coriolis[i]);
			}
			else
			{
				robots[i]->updateInverseInertia();
			}
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
					robots[1]->_M(j,j) += 0.1;
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