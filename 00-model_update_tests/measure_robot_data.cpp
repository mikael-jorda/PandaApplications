// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "filters/ButterworthFilter.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"

#include <iostream>
#include <string>
#include <fstream>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "resources/panda_arm.urdf";
const string robot_name = "PANDA";


#define DEBUG                           0
#define GO_TO_INITIAL                   1
#define EXPERIMENT                      2

int state = GO_TO_INITIAL;


// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

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

unsigned long long controller_counter = 0;

const bool flag_simulation = false;
// const bool flag_simulation = true;

const string prefix_path = "../../00-experiment_pose_control/data_files/measure_robot_data/";

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplications::simviz_panda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplications::simviz_panda::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplications::simviz_panda::actuators::fgc";
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

	robot->updateModel();


	// prepare controller	
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	VectorXd initial_q = VectorXd::Zero(dof);
	// initial_q << 0.0, 25.0, 0.0, -110.0, 0.0, 125.0, 45.0;
	// initial_q *= M_PI/180.0;

	initial_q << 0.0,0.46742,0.0,-1.26446,0.0,1.74011,0.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = true;

	joint_task->_otg->setMaxVelocity(M_PI/4);

	joint_task->_kp = 200.0;
	joint_task->_kv = 15.0;
	joint_task->_ki = 5.0;

	joint_task->_desired_position = initial_q;

	// posori task
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.107);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_use_interpolation_flag = false;

	Vector3d x_init = Vector3d::Zero();
	Matrix3d R_init = Matrix3d::Identity();

	double initial_time = 0;

	posori_task->_desired_position = x_init; 
	posori_task->_desired_orientation = R_init; 
	posori_task->_ki_pos = 5.0;
	posori_task->_ki_ori = 5.0;

	// prepare file to write data
	string filename = prefix_path + "data_" + currentDateTime();
	ofstream data_file;
	data_file.open(filename);

	data_file << "joint positions[7]\tjoint velocities[7]\tcurrent_position[3]\tcurrent_velocity[3]\tcurrent_angular_velocity[3]\t" <<
				"mass_matrix[49](col_based)\ttask_jacobian[42](col_based_transposed)\n";

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

		if(state == GO_TO_INITIAL)
		{
			joint_task->updateTaskModel(MatrixXd::Identity(dof,dof));

			robot->_M += 0.1 * MatrixXd::Identity(dof,dof);
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			if((joint_task->_desired_position - joint_task->_current_position).norm() < 1e-1)
			{
				posori_task->reInitializeTask();

				state = EXPERIMENT;
				initial_time = current_time;
			}
		}

		else if(state == EXPERIMENT)
		{

			posori_task->updateTaskModel(MatrixXd::Identity(dof,dof));
			posori_task->computeTorques(posori_task_torques);


			data_file << robot->_q.transpose() << '\t' << robot->_dq.transpose() << '\t' << posori_task->_current_position.transpose() << '\t' << 
				posori_task->_current_velocity.transpose() << '\t' << posori_task->_current_angular_velocity.transpose() << '\t';

			for(int k=0 ; k<dof ; k++)
			{
				data_file << robot->_M.col(k).transpose() << '\t';
			}
			for(int k=0 ; k<dof ; k++)
			{
				data_file << posori_task->_jacobian.col(k).transpose() << '\t';
			}
			data_file << endl;

			// compute torques
			command_torques.setZero();
		}
		else
		{
			command_torques.setZero();
		}

		// command_torques.setZero(dof);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		prev_time = current_time;
		controller_counter++;
	}

	command_torques.setZero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	data_file.close();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

