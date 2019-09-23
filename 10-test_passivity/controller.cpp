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

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

#define MOVE_TO_INITIAL      0
#define MOVE_TO_CONTACT      1
#define FORCE_CONTROL        2

int state = MOVE_TO_INITIAL;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;

std::string SENSED_FORCE_MOMENT_KEY; 

// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

// gains
const string KP_JOINT_KEY = "sai2::PandaApplication::controller:kp_joint";
const string KV_JOINT_KEY = "sai2::PandaApplication::controller:kv_joint";
const string KP_POS_KEY = "sai2::PandaApplication::controller:kp_pos";
const string KV_POS_KEY = "sai2::PandaApplication::controller:kv_pos";
const string KP_ORI_KEY = "sai2::PandaApplication::controller:kp_ori";
const string KV_ORI_KEY = "sai2::PandaApplication::controller:kv_ori";

// desired force
const string DESIRED_EE_FORCE_KEY = "sai2::PandaApplication::controller:desried_ee_force";

// logging quantities
const string LOGGING_PO_KEY = "sai2::PandaApplication::logging::passivity_observer";
const string LOGGING_RC_INV_KEY = "sai2::PandaApplication::logging::Rc_inv";
const string LOGGING_DESIRED_FORCE_KEY = "sai2::PandaApplication::logging::desired_force";
const string LOGGING_SENSED_FORCE_KEY = "sai2::PandaApplication::logging::sensed_force";


unsigned long long controller_counter = 0;
int olfc_counter = 1500;

const bool flag_simulation = false;
// const bool flag_simulation = true;

const bool inertia_regularization = true;

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

		SENSED_FORCE_MOMENT_KEY = "sai2::PandaApplication::sensors::force_moment";

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

		SENSED_FORCE_MOMENT_KEY = "sai2::ATIGamma_Sensor::force_torque";	

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
	auto joint_task = new Sai2Primitives::JointTask(robot);

	MatrixXd N_prec = MatrixXd::Identity(robot->dof(), robot->dof());
	VectorXd joint_task_torques = VectorXd::Zero(robot->dof());
	joint_task->_kp = 150.0;
	joint_task->_kv = 15.0;
	joint_task->_ki = 20.0;
	redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
	redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));

	VectorXd command_torques = VectorXd::Zero(robot->dof());

	// Eigen::VectorXd goal_position(robot->dof());
	// goal_position << 0, 30, 0, -90, 0, 120, 0;
	// goal_position *= M_PI/180.0;
	// joint_task->_desired_position = goal_position;

	joint_task->_desired_position << 0.65, 0.30, 0.0, -1.30, 0.0, 1.6, -0.80;

	// posori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.2);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(robot->dof());

	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 17.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 23.0;
	redis_client.set(KP_POS_KEY, to_string(posori_task->_kp_pos));
	redis_client.set(KV_POS_KEY, to_string(posori_task->_kv_pos));
	redis_client.set(KP_ORI_KEY, to_string(posori_task->_kp_ori));
	redis_client.set(KV_ORI_KEY, to_string(posori_task->_kv_ori));

	posori_task->_otg->setMaxLinearVelocity(0.05);

	// Vector3d sensed_force = Vector3d::Zero();
	// Vector3d sensed_moment = Vector3d::Zero();

	VectorXd sensed_force_moment = VectorXd::Zero(6);
	VectorXd force_bias = VectorXd::Zero(6);
	force_bias << 0.119518,   0.0137842,     1.50978, -0.00386726, -0.00689004, 0.000847295;

	// double ee_mass = 0.0594724;
	double ee_mass = 0.13;
	Vector3d p_sensor_eecom = Vector3d(0.00527075, -0.00639089, 3.55516e-05);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) {
	// wait for next scheduled loop
	timer.waitForNextLoop();
	double time = timer.elapsedTime() - start_time;

	// read robot state from redis
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

	sensed_force_moment = redis_client.getEigenMatrixJSON(SENSED_FORCE_MOMENT_KEY);
	if(!flag_simulation)
	{
		// remove bias and ee mass
		sensed_force_moment -= force_bias;

		Matrix3d R_sensor;
		robot->rotation(R_sensor, "link7");
		Vector3d globalz_loc_frame = R_sensor.transpose().col(2);

		sensed_force_moment.head(3) -= ee_mass * 9.81 * globalz_loc_frame;
		sensed_force_moment.tail(3) -= ee_mass * 9.81 * p_sensor_eecom.cross(globalz_loc_frame);

	}

	posori_task->updateSensedForceAndMoment(sensed_force_moment.head(3), sensed_force_moment.tail(3));


	// update robot model
	if(flag_simulation)
	{
		robot->updateModel();
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
		// cout << robot->_M << endl;
		// robot->_M = Eigen::MatrixXd::Identity(robot->dof(), robot->dof());
		robot->_M_inv = robot->_M.inverse();
	}

	if(state == MOVE_TO_INITIAL)
	{
		// update tasks model
		joint_task->updateTaskModel(N_prec);

		// compute torques
		joint_task->computeTorques(joint_task_torques);

		command_torques = joint_task_torques;

		if((joint_task->_desired_position - joint_task->_current_position).norm() < 0.1)
		{
			joint_task->_kp = 0.0;
			joint_task->_ki = 0.0;
			joint_task->_kv = 5.0;
			redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
			redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));
			posori_task->reInitializeTask();
			posori_task->_desired_position += Vector3d(0,0,-0.1);
			state = MOVE_TO_CONTACT;
		}
	}
	else if(state == MOVE_TO_CONTACT)
	{
		// update tasks model
		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);

		// compute torques
		posori_task->computeTorques(posori_task_torques);

		joint_task->computeTorques(joint_task_torques);

		command_torques = posori_task_torques + joint_task_torques;

		// cout << sensed_force.transpose() << endl;

		if( sensed_force_moment(2) > 5 )
		{
			cout << "open loop force control start" << endl;
			posori_task->setForceAxis(Vector3d(0,0,1));
			posori_task->_desired_force = Vector3d(0,0,-0);
			redis_client.set(DESIRED_EE_FORCE_KEY, to_string(-0));
			// posori_task->setClosedLoopForceControl();
			state = FORCE_CONTROL;
		}
	}
	else if(state == FORCE_CONTROL)
	{


		N_prec.setIdentity();
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);

		// compute torques
		posori_task->_desired_force(2) = stod(redis_client.get(DESIRED_EE_FORCE_KEY));
		posori_task->computeTorques(posori_task_torques);

		joint_task->computeTorques(joint_task_torques);

		command_torques = posori_task_torques + joint_task_torques;	

		if(olfc_counter == 0)
		{
			cout << "closed loop force control start" << endl;
			// posori_task->_passivity_enabled = false;
			posori_task->setClosedLoopForceControl();
			olfc_counter--;
			posori_task->_kp_force = 0.3;
			posori_task->_ki_force = 2.7;
		}	
		else
		{
			olfc_counter--;
		}
	}

	if(controller_counter % 100 == 0)
	{
		cout << "sensed force world frame : " << posori_task->_sensed_force.transpose() << endl;
		cout << endl;
	}

	// send to redis
	// command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// logging to redis
	redis_client.set(LOGGING_PO_KEY, to_string(posori_task->_passivity_observer + posori_task->_stored_energy));
	redis_client.set(LOGGING_RC_INV_KEY, to_string(posori_task->_Rc_inv));
	redis_client.setEigenMatrixJSON(LOGGING_SENSED_FORCE_KEY, posori_task->_sensed_force);
	redis_client.setEigenMatrixJSON(LOGGING_DESIRED_FORCE_KEY, posori_task->_desired_force);

	controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
