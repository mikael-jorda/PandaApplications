// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "filters/ButterworthFilter.h"
#include "KalmanFilter.h"

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

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// logger
string LOG_Q_KEY = "sai2::PandaApplication::logger::q";
string LOG_DQ_DRIVER_KEY = "sai2::PandaApplication::logger::dq_driver";

string LOG_DQ_DRIVER_FILTERED_KEY = "sai2::PandaApplication::logger::dq_driver_filtered";
string LOG_DQ_FROM_QDIFF_FILTERED_KEY = "sai2::PandaApplication::logger::dq_driver_from_q_diff_filtered";
string LOG_DDQ_FROM_DQ_DRIVER_FILTERED_KEY = "sai2::PandaApplication::logger::ddq_driver_from_dq_driver_filtered";
string LOG_DDQ_FROM_DQ_DIFF_FILTERED_KEY = "sai2::PandaApplication::logger::ddq_driver_from_dq_diff_filtered";

string LOG_Q_KALMAN_KEY = "sai2::PandaApplication::logger::q_kalman";
string LOG_DQ_KALMAN_KEY = "sai2::PandaApplication::logger::dq_kalman";
string LOG_DDQ_KALMAN_KEY = "sai2::PandaApplication::logger::ddq_kalman";

unsigned long long controller_counter = 0;

const bool flag_simulation = false;
// const bool flag_simulation = true;

const bool inertia_regularization = true;

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
	int dof = robot->dof();
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	VectorXd q = VectorXd::Zero(dof);
	VectorXd q_prev = VectorXd::Zero(dof);
	VectorXd dq_driver = VectorXd::Zero(dof);
	VectorXd dq_driver_prev = VectorXd::Zero(dof);
	VectorXd dq_from_q_diff = VectorXd::Zero(dof);
	VectorXd dq_from_q_diff_prev = VectorXd::Zero(dof);
	VectorXd ddq_from_dq_driver = VectorXd::Zero(dof);
	VectorXd ddq_from_dq_diff = VectorXd::Zero(dof);

	VectorXd dq_driver_filtered = VectorXd::Zero(dof);
	VectorXd dq_from_q_diff_filtered = VectorXd::Zero(dof);
	VectorXd ddq_from_dq_driver_filtered = VectorXd::Zero(dof);
	VectorXd ddq_from_dq_diff_filtered = VectorXd::Zero(dof);

	VectorXd q_kalman = VectorXd::Zero(dof);
	VectorXd dq_kalman = VectorXd::Zero(dof);
	VectorXd ddq_kalman = VectorXd::Zero(dof);

	// create filters
	// - butterworth
	auto lowpass_filter_dq_driver = new ButterworthFilter(dof, 0.07);
	auto lowpass_filter_dq_diff = new ButterworthFilter(dof, 0.015);
	auto lowpass_filter_ddq_from_dq_driver = new ButterworthFilter(dof, 0.015);
	auto lowpass_filter_ddq_from_dq_diff = new ButterworthFilter(dof, 0.005);

	// - kalman
	double dt = 0.001;
	MatrixXd F = MatrixXd::Identity(21,21);
	F.block<7,7>(0,7) = dt * MatrixXd::Identity(7,7);
	F.block<7,7>(7,14) = dt * MatrixXd::Identity(7,7);

	MatrixXd H = MatrixXd::Zero(7,21);
	H.block<7,7>(0,0) = MatrixXd::Identity(7,7);

	MatrixXd Q = MatrixXd::Zero(21,21);
	Q.block<7,7>(14,14) = 1000.0 * MatrixXd::Identity(7,7);

	MatrixXd R = 0.1 * MatrixXd::Identity(7,7);

	auto kalman_filter = new KalmanFilters::KalmanFilter(dt, F, H, Q, R);
	VectorXd x0_kalman = VectorXd::Zero(21);
	x0_kalman.head(7) = robot->_q;
	kalman_filter->init(x0_kalman);

	VectorXd kalman_state = VectorXd::Zero(21);

	// redis setup
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, dq_driver);

	redis_client.addEigenToWriteCallback(0, LOG_Q_KEY, q);
	redis_client.addEigenToWriteCallback(0, LOG_DQ_DRIVER_KEY, dq_driver);

	redis_client.addEigenToWriteCallback(0, LOG_DQ_DRIVER_FILTERED_KEY, dq_driver_filtered);
	redis_client.addEigenToWriteCallback(0, LOG_DQ_FROM_QDIFF_FILTERED_KEY, dq_from_q_diff_filtered);
	redis_client.addEigenToWriteCallback(0, LOG_DDQ_FROM_DQ_DRIVER_FILTERED_KEY, ddq_from_dq_driver_filtered);
	redis_client.addEigenToWriteCallback(0, LOG_DDQ_FROM_DQ_DIFF_FILTERED_KEY, ddq_from_dq_diff_filtered);

	redis_client.addEigenToWriteCallback(0, LOG_Q_KALMAN_KEY, q_kalman);
	redis_client.addEigenToWriteCallback(0, LOG_DQ_KALMAN_KEY, dq_kalman);
	redis_client.addEigenToWriteCallback(0, LOG_DDQ_KALMAN_KEY, ddq_kalman);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	double prev_time = 0;
	double time = 0;
	bool fTimerDidSleep = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		prev_time = time;
		time = timer.elapsedTime() - start_time;
		// double dt = time - prev_time;

		// read robot state from redis
		redis_client.executeReadCallback(0);

		dq_from_q_diff = (q - q_prev)/dt;
		ddq_from_dq_driver = (dq_driver - dq_driver_prev)/dt;
		ddq_from_dq_diff = (dq_from_q_diff - dq_from_q_diff_prev)/dt;

		VectorXd dq_driver_filtered_tmp = lowpass_filter_dq_driver->update(dq_driver);
		VectorXd dq_from_q_diff_filtered_tmp = lowpass_filter_dq_diff->update(dq_from_q_diff);
		VectorXd ddq_from_dq_driver_filtered_tmp = lowpass_filter_ddq_from_dq_driver->update(ddq_from_dq_driver);
		VectorXd ddq_from_dq_diff_filtered_tmp = lowpass_filter_ddq_from_dq_diff->update(ddq_from_dq_diff);
		dq_driver_filtered = dq_driver_filtered_tmp;
		dq_from_q_diff_filtered = dq_from_q_diff_filtered_tmp;
		ddq_from_dq_driver_filtered = ddq_from_dq_driver_filtered_tmp;
		ddq_from_dq_diff_filtered = ddq_from_dq_diff_filtered_tmp;

		if(dq_driver_filtered(2) > 3)
		{
			cout << dq_driver_filtered.transpose() << endl;
			cout << dq_driver_filtered.norm() << endl;
			cout << endl;
		}

		kalman_filter->update(q);
		kalman_state = kalman_filter->getState();
		q_kalman = kalman_state.segment<7>(0);
		dq_kalman = kalman_state.segment<7>(7);
		ddq_kalman = kalman_state.segment<7>(14);

		redis_client.executeWriteCallback(0);

		q_prev = q;
		dq_driver_prev = dq_driver;
		dq_from_q_diff_prev = dq_from_q_diff;

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
