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

const string robot_file = "../resources/01bis-cartesian_hold_position/panda_arm.urdf";
const string robot_name = "PANDA";

#define GO_TO_INIT_CONFIG       0
#define HOLD_CARTESIAN_POS      1

int state = GO_TO_INIT_CONFIG;
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

const string DESIRED_POS_KEY = "sai2::PandaApplication::controller::desired_position";

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

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd joint_task_torques_np = VectorXd::Zero(dof);
	joint_task->_kp = 150.0;
	joint_task->_kv = 25.0;
	joint_task->_ki = 150.0;
	redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
	redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));

	joint_task->_max_velocity = 30.0 * M_PI/180.0;

	VectorXd q_init_desired = VectorXd::Zero(dof);
	// q_init_desired << 0, 0, 0, -10, 0, 90, 0;
	q_init_desired << 50, 15, 0, -95, 0, 125, 0;
	q_init_desired *= M_PI/180;
	joint_task->_goal_position = q_init_desired;

	// pos task
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0.0,0.0,0.2213);
	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);

	VectorXd pos_task_torques = VectorXd::Zero(dof);

	pos_task->_max_velocity = 0.1;

	Vector3d x_goal = pos_task->_current_position;

	pos_task->_kp = 300.0;
	pos_task->_kv = 25.0;
	pos_task->_ki = 100.0;
	redis_client.set(KP_POS_KEY, to_string(pos_task->_kp));
	redis_client.set(KV_POS_KEY, to_string(pos_task->_kv));

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
			joint_task->_kp = stod(redis_client.get(KP_JOINT_KEY));
			joint_task->_kv = stod(redis_client.get(KV_JOINT_KEY));
			pos_task->_kp = stod(redis_client.get(KP_POS_KEY));
			pos_task->_kv = stod(redis_client.get(KV_POS_KEY));
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY) + M_reg;
			robot->_M_inv = robot->_M.inverse();

			coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
		}

		if(state == GO_TO_INIT_CONFIG)
		{
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			if((joint_task->_current_position - joint_task->_goal_position).norm() < 0.01)
			{
				joint_task->_ki = 0.0;
				joint_task->_kp = 0.0;
				joint_task->_kv = 10.0;
				redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
				redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));

				joint_task->reInitializeTask();
				pos_task->reInitializeTask();
				q_init_desired = robot->_q;
				robot->position(x_goal, link_name, pos_in_link);
				redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, x_goal);
				state_init_counter = controller_counter;
				state = HOLD_CARTESIAN_POS;
			}
		}

		else if(state == HOLD_CARTESIAN_POS)
		{
			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;
			joint_task->updateTaskModel(N_prec);

			//update desired position
			x_goal = redis_client.getEigenMatrixJSON(DESIRED_POS_KEY);
			pos_task->_goal_position = x_goal;

			// compute torques
			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = pos_task_torques + joint_task_torques + coriolis;

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

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
