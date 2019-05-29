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

#define CONTROLLER_1       0      // constant stiffness by choosing gains in the direction of the error
#define CONTROLLER_2       1      // constant stiffness by choosing gains depending on mass matrix
#define CONTROLLER_3       2      // test max gains and tracking error in static case
#define CONTROLLER_4       3      // Constant stiffness orientation

int state = CONTROLLER_3;
unsigned long long state_init_counter = 0;

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
const string KP_JOINT_KEY = "sai2::PandaApplication::controller:kp_joint";
const string KV_JOINT_KEY = "sai2::PandaApplication::controller:kv_joint";
const string KP_POS_KEY = "sai2::PandaApplication::controller:kp_pos";
const string KV_POS_KEY = "sai2::PandaApplication::controller:kv_pos";
const string KP_ORI_KEY = "sai2::PandaApplication::controller:kp_ori";
const string KV_ORI_KEY = "sai2::PandaApplication::controller:kv_ori";

const string DESIRED_POS_KEY = "sai2::PandaApplication::controller::desired_position";

unsigned long long controller_counter = 0;

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
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";		

		GRIPPER_MODE_KEY  = "sai2::FrankaPanda::gripper::mode"; // m for move and g for graps
		GRIPPER_MAX_WIDTH_KEY  = "sai2::FrankaPanda::gripper::max_width";
		GRIPPER_CURRENT_WIDTH_KEY  = "sai2::FrankaPanda::gripper::current_width";
		GRIPPER_DESIRED_WIDTH_KEY  = "sai2::FrankaPanda::gripper::desired_width";
		GRIPPER_DESIRED_SPEED_KEY  = "sai2::FrankaPanda::gripper::desired_speed";
		GRIPPER_DESIRED_FORCE_KEY  = "sai2::FrankaPanda::gripper::desired_force";
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

	// prepare controller	
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 150.0;
	joint_task->_kv = 5.0;

	// posori task
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0.0,0.0,0.15);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	Vector3d x_init = posori_task->_current_position;

	VectorXd posori_task_torques = VectorXd::Zero(dof);

	posori_task->_use_interpolation_flag = false;

	double circle_radius = 0.05;
	double circle_period = 3.0;

	double oscillation_amplitude = M_PI/6;
	double oscillation_period = 3.0;

	redis_client.set(KP_POS_KEY, to_string(posori_task->_kp_pos));
	redis_client.set(KP_ORI_KEY, to_string(posori_task->_kp_ori));
	redis_client.set(KV_POS_KEY, to_string(posori_task->_kv_pos));
	redis_client.set(KV_ORI_KEY, to_string(posori_task->_kv_ori));

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

		if(state == CONTROLLER_1)
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			//update desired position
			posori_task->_desired_position(0) = x_init(0) + circle_radius*sin(2*M_PI*current_time/circle_period);
			posori_task->_desired_position(1) = x_init(1) + circle_radius*(1 - cos(2*M_PI*current_time/circle_period));

			// compute torques fake for update state of posori controller
			posori_task->computeTorques(posori_task_torques);

			// update the gains
			posori_task->_kp_ori = 400.0;
			posori_task->_kv_ori = 40.0;

			Vector3d pos_error = posori_task->_desired_position - posori_task->_current_position;
			Vector3d u = pos_error/pos_error.norm();
			double effective_mass_inv = u.dot(posori_task->_Lambda.block<3,3>(0,0).inverse()*u);

			posori_task->_kp_pos = 100.0 * effective_mass_inv;
			posori_task->_kv_pos = 2 * sqrt(posori_task->_kp_pos);

			// compute torques real
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
		}

		else if(state == CONTROLLER_2)
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			//update desired position
			posori_task->_desired_position(0) = x_init(0) + circle_radius*sin(2*M_PI*current_time/circle_period);
			posori_task->_desired_position(1) = x_init(1) + circle_radius*(1 - cos(2*M_PI*current_time/circle_period));

			// compute torques fake for update state of posori controller
			posori_task->computeTorques(posori_task_torques);

			// update the gains
			posori_task->_kp_ori = 400.0;
			posori_task->_kv_ori = 40.0;

			posori_task->_use_isotropic_gains = false;

			posori_task->_kp_pos_vec = 100.0 * Vector3d(1.0/posori_task->_Lambda(0,0), 1.0/posori_task->_Lambda(1,1), 1.0/posori_task->_Lambda(2,2));
			posori_task->_kv_pos_vec = 2 * Vector3d(sqrt(posori_task->_kp_pos_vec(0)), sqrt(posori_task->_kp_pos_vec(1)), sqrt(posori_task->_kp_pos_vec(2)));

			// compute torques real
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
		}

		else if(state == CONTROLLER_3)
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// update the gains
			posori_task->_kp_pos = stod(redis_client.get(KP_POS_KEY));
			posori_task->_kv_pos = stod(redis_client.get(KV_POS_KEY));
			posori_task->_kp_ori = stod(redis_client.get(KP_ORI_KEY));
			posori_task->_kv_ori = stod(redis_client.get(KV_ORI_KEY));
			
			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;

			if(controller_counter % 100 == 0)
			{
				cout << "pos error : " << (posori_task->_current_position - posori_task->_desired_position).transpose() << endl;
				cout << "ori error : " << (posori_task->_orientation_error).transpose() << endl;
				cout << endl;
			}
		}

		else if(state == CONTROLLER_4)
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques fake for update state of posori controller
			posori_task->computeTorques(posori_task_torques);

			// update the gains
			posori_task->_kp_pos = 150.0;
			posori_task->_kv_pos = 10.0;

			posori_task->_use_isotropic_gains = false;

			posori_task->_kp_ori_vec = 100.0 * Vector3d(1.0/posori_task->_Lambda(3,3), 1.0/posori_task->_Lambda(4,4), 1.0/posori_task->_Lambda(5,5));
			posori_task->_kv_ori_vec = 2 * Vector3d(sqrt(posori_task->_kp_ori_vec(0)), sqrt(posori_task->_kp_ori_vec(1)), sqrt(posori_task->_kp_ori_vec(2)));
			
			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
		}

		else
		{
			command_torques.setZero(dof);
		}

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
