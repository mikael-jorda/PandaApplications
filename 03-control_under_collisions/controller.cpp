// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

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

const string robot_file = "../resources/03-control_under_collisions/panda_arm.urdf";
const string robot_name = "PANDA";

#define HOLD_CARTESIAN_POS      0

int state = HOLD_CARTESIAN_POS;

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

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool inertia_regularization = true;

// logging
const string VEL_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::velocity_based_observer";
const string MOM_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::momentum_based_observer";
const string CONTACT_COMPENSATION_TORQUES_KEY = "sai2::PandaApplication::controller::logging::contact_compensation_torques";
const string DESIRED_POS_KEY = "sai2::PandaApplication::controller::logging::desired_position";
const string CURRENT_POS_KEY = "sai2::PandaApplication::controller::logging::current_position";
const string COMMAND_TORQUES_LOGGING_KEY = "sai2::PandaApplication::controller::logging::command_torques";

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
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint taks
	auto joint_task = new Sai2Primitives::JointTask(robot);

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 15.0;
	redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
	redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));

	joint_task->_max_velocity = 15.0 * M_PI/180.0;

	// posori task
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.2);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);

	VectorXd posori_task_torques = VectorXd::Zero(dof);

	posori_task->_max_velocity = 0.05;

	posori_task->_goal_position(1) += 0.15;

	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 17.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 23.0;
	redis_client.set(KP_POS_KEY, to_string(posori_task->_kp_pos));
	redis_client.set(KV_POS_KEY, to_string(posori_task->_kv_pos));
	redis_client.set(KP_ORI_KEY, to_string(posori_task->_kp_ori));
	redis_client.set(KV_ORI_KEY, to_string(posori_task->_kv_ori));

	// prepare observers
	VectorXd gravity(dof), coriolis(dof);
	VectorXd r_vel = VectorXd::Zero(dof);
	VectorXd r_mom = VectorXd::Zero(dof);
	VectorXd p = VectorXd::Zero(dof);
	VectorXd beta = VectorXd::Zero(dof);
	VectorXd integral_vel = VectorXd::Zero(dof);
	VectorXd integral_mom = VectorXd::Zero(dof);
	MatrixXd K0 = 10*MatrixXd::Identity(dof,dof);
	MatrixXd M_dot = MatrixXd::Identity(dof,dof);
	MatrixXd M_prev = MatrixXd::Identity(dof,dof);
	bool first_iteration = true;

	// torques to compensate contact at task level
	VectorXd contact_compensation_torques = VectorXd::Zero(dof);

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
			robot->gravityVector(gravity);
			robot->coriolisForce(coriolis);
		}
		else
		{
			joint_task->_kp = stod(redis_client.get(KP_JOINT_KEY));
			joint_task->_kv = stod(redis_client.get(KV_JOINT_KEY));
			posori_task->_kp_pos = stod(redis_client.get(KP_POS_KEY));
			posori_task->_kv_pos = stod(redis_client.get(KV_POS_KEY));
			posori_task->_kp_ori = stod(redis_client.get(KP_ORI_KEY));
			posori_task->_kv_ori = stod(redis_client.get(KV_ORI_KEY));
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();

			gravity = redis_client.getEigenMatrixJSON(ROBOT_GRAVITY_KEY);
			coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
		}
		M_dot = (robot->_M - M_prev)/dt;

		if(state == HOLD_CARTESIAN_POS)
		{
			// update tasks model
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute velocity based observer
			if(!first_iteration)
			{
				integral_vel += robot->_M_inv * (command_torques + contact_compensation_torques - coriolis + r_vel) * dt;
				r_vel = K0 * (robot->_dq - integral_vel);
			}

			// compute momentum based observer
			if(!first_iteration)
			{
				beta = coriolis;
				integral_mom += (command_torques - beta + r_mom) * dt;
				r_mom = K0 * (robot->_M * robot->_q - integral_mom);
			}

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			// contact_compensation_torques.setZero();
			contact_compensation_torques = posori_task->_projected_jacobian.transpose() * posori_task->_Jbar.transpose() * r_vel;

			command_torques = posori_task_torques + joint_task_torques + coriolis - contact_compensation_torques;

			if(controller_counter == 4000)
			{
				posori_task->_goal_position(1) -= 0.2;
			}

		}
		else
		{
			command_torques.setZero(dof);
		}

		// send to redis
		// command_torques.setZero(dof);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// log data
		redis_client.setEigenMatrixJSON(VEL_OBSERVER_LOGGING_KEY, r_vel);
		redis_client.setEigenMatrixJSON(MOM_OBSERVER_LOGGING_KEY, r_mom);
		redis_client.setEigenMatrixJSON(CONTACT_COMPENSATION_TORQUES_KEY, contact_compensation_torques);
		redis_client.setEigenMatrixJSON(COMMAND_TORQUES_LOGGING_KEY, command_torques);
		redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, posori_task->_desired_position);
		redis_client.setEigenMatrixJSON(CURRENT_POS_KEY, posori_task->_current_position);

		if(controller_counter % 500 == 0)
		{
			cout << "velocity based observer :\n" << r_vel.transpose() << endl;
			cout << "momentum based observer :\n" << r_mom.transpose() << endl;
			cout << endl;
		}

		prev_time = current_time;
		controller_counter++;

		if(first_iteration)
		{
			first_iteration = false;
		}

	}

double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
