#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
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

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

#define JOINT_CONTROL           0
#define CARTESIAN_CONTROL       1
#define FOLLOW_CIRCLE           2

int state = JOINT_CONTROL;
int previous_state = JOINT_CONTROL;
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

// gains
const string KP_JOINT_KEY = "sai2::PandaApplication::controller::kp_joint";
const string KV_JOINT_KEY = "sai2::PandaApplication::controller::kv_joint";
const string KP_POS_KEY = "sai2::PandaApplication::controller::kp_pos";
const string KV_POS_KEY = "sai2::PandaApplication::controller::kv_pos";

// controller state
const string STATE_KEY = "sai2::PandaApplication::controller::state";

unsigned long long controller_counter = 0;

const bool flag_simulation = false;
// const bool flag_simulation = true;

const bool inertia_regularization = true;
// const bool inertia_regularization = false;

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Bonnie::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";		
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

	redis_client.set(STATE_KEY, to_string(state));

	// prepare controller	
	int dof = robot->dof();
	MatrixXd M_reg = MatrixXd::Zero(dof,dof);
	if(inertia_regularization)
	{
		M_reg(4,4) = 0.07;
		M_reg(5,5) = 0.07;
		M_reg(6,6) = 0.07;
	}
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_isotropic_gains = false;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd joint_task_torques_np = VectorXd::Zero(dof);

	joint_task->_kp_vec = 150.0*VectorXd::Ones(dof);
	joint_task->_kv_vec = 25.0*VectorXd::Ones(dof);
	redis_client.setEigenMatrixJSON(KP_JOINT_KEY, joint_task->_kp_vec);
	redis_client.setEigenMatrixJSON(KV_JOINT_KEY, joint_task->_kv_vec);


	VectorXd q_init_desired = VectorXd::Zero(dof);
	// q_init_desired << 0, 0, 0, -10, 0, 90, 0;
	// q_init_desired << 50, 15, 0, -95, 0, 125, 0;
	// q_init_desired *= M_PI/180;
	q_init_desired << -0.878802,-0.81999,1.2349,-2.01274,-2.38334,1.51654,1.22981;
	joint_task->_desired_position = q_init_desired;

	// pos task
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.2213);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_use_interpolation_flag = false;

	Vector3d initial_pos = posori_task->_current_position;
	robot->position(initial_pos, link_name, pos_in_link);
	double init_time = 0;
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	posori_task->_use_isotropic_gains = false;
	posori_task->_kp_pos_vec = 300.0 * Vector3d::Ones();
	posori_task->_kv_pos_vec = 25.0 * Vector3d::Ones();
	posori_task->_kp_ori_vec = 300.0 * Vector3d::Ones();
	posori_task->_kv_ori_vec = 25.0 * Vector3d::Ones();
	// redis_client.set(KP_POS_KEY, to_string(posori_task->_kp_pos));
	// redis_client.set(KV_POS_KEY, to_string(posori_task->_kv_pos));
	redis_client.setEigenMatrixJSON(KP_POS_KEY, posori_task->_kp_pos_vec);
	redis_client.setEigenMatrixJSON(KV_POS_KEY, posori_task->_kv_pos_vec);

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
			// joint_task->_kp = stod(redis_client.get(KP_JOINT_KEY));
			// joint_task->_kv = stod(redis_client.get(KV_JOINT_KEY));
			joint_task->_kp_vec = redis_client.getEigenMatrixJSON(KP_JOINT_KEY);
			joint_task->_kv_vec = redis_client.getEigenMatrixJSON(KV_JOINT_KEY);
			// posori_task->_kp_pos = stod(redis_client.get(KP_POS_KEY));
			// posori_task->_kv_pos = stod(redis_client.get(KV_POS_KEY));
			posori_task->_kp_pos_vec = redis_client.getEigenMatrixJSON(KP_POS_KEY);
			posori_task->_kv_pos_vec = redis_client.getEigenMatrixJSON(KV_POS_KEY);
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY) + M_reg;
			robot->_M_inv = robot->_M.inverse();

			coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
		}

		// read new state from redis
		state = stoi(redis_client.get(STATE_KEY));
		if(state != previous_state)
		{
			cout << "state changed" << endl;
			cout << posori_task->_current_position.transpose() << endl;
			joint_task->reInitializeTask();
			posori_task->reInitializeTask();
			cout << posori_task->_current_position.transpose() << endl;
			initial_pos = posori_task->_current_position;
			init_time = current_time;
		}

		if(state == JOINT_CONTROL)
		{
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;
		}

		else if(state == CARTESIAN_CONTROL)
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
		}

		else if(state == FOLLOW_CIRCLE)
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// set desired position
			posori_task->_desired_position(0) = initial_pos(0) + 0.05 * sin(2*M_PI*0.2*(current_time - init_time));
			posori_task->_desired_position(2) = initial_pos(2) + 0.05 * (1-cos(2*M_PI*0.2*(current_time - init_time)));

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

		previous_state = state;
		prev_time = current_time;
		controller_counter++;
	}

	// send zero torques to redis and then exit
	command_torques.setZero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
