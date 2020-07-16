// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <queue>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/pbot.urdf";

#define MOVE_TO_CONTACT      0
#define FORCE_CONTROL        1

int state = MOVE_TO_CONTACT;

// redis keys:
// - read:
const string JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
const string JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
const string JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

const string SENSED_FORCE_FROM_SIM_KEY = "sai2::PandaApplication::sensors::force";

// logging
const string DESIRED_EE_FORCE_KEY = "sai2::PandaApplication::controller:desried_ee_force";
const string SENSED_EE_FORCE_KEY = "sai2::PandaApplication::controller:sensed_ee_force";
const string RC_KEY = "sai2::PandaApplication::controller:Rc";
const string FORWARD_PO_KEY = "sai2::PandaApplication::controller:forward_PO";
const string BACKWARD_PO_KEY = "sai2::PandaApplication::controller:backward_PO";
const string STORED_ENERGY_KEY = "sai2::PandaApplication::controller:E_stored";
const string CORRECTION_ENERGY_BACKWARD_KEY = "sai2::PandaApplication::controller:E_correction_backward";
const string ENERGY_TO_DISSIPATE_BACKWARD_KEY = "sai2::PandaApplication::controller:E_to_dissipate_backward";
const string CORRECTION_ENERGY_FORWARD_KEY = "sai2::PandaApplication::controller:E_correction_forward";
const string VC_KEY = "sai2::PandaApplication::controller:vc";
const string ALPHA_KEY = "sai2::PandaApplication::controller:alpha";

const string CONTROLLER_RUNNING_KEY = "sai2::PandaApplication::controller::flag_running";

unsigned long long controller_counter = 0;
int olfc_counter = 200;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool inertia_regularization = true;

int main() {

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
	int dof = robot->dof();
	VectorXd initial_q = robot->_q;
	robot->updateModel();
	VectorXd command_torques = VectorXd::Zero(dof);

	// prepare controller
	double Fd = -5.0;
	double Fcmd = 0;
	double vr = 0;
	double kp_force = 1.5;
	double ki_force = 2.5;
	double kv_force = 15.0;
	double k_ff = 1.0;

	Vector3d sensed_force = Vector3d::Zero();
	double Fs = 0;

	double integrated_force_error = 0;

	// passivity
	double backward_PO = 0;
	double forward_PO = 0;
	double E_stored = 0;
	double E_correction_backwards = 0;
	double E_correction_forward = 0;
	double E_to_dissipate_backward = 0;

	double vc = 0;
	double Rc = 1.0;
	double Rpc = 0.0;
	double gain_scaling = Rc/(Rc + Rpc);

	double alpha_forward = 0;
	double alpha_max = 25.0;

	const int windowed_PO_size = 25;
	queue<double> windowed_PO;
	queue<double> windowed_E_correction;

	// force chirp
	double freq_init = 0.0;
	double freq_final = 10.0;
	double freq = freq_init;
	double freq_increment_per_second = 0.5;
	double amplitude = 0.5;

	double t0 = 0;

	// setup redis exchange
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToReadCallback(0, SENSED_FORCE_FROM_SIM_KEY, sensed_force);

	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.addDoubleToWriteCallback(0, BACKWARD_PO_KEY, backward_PO);
	redis_client.addDoubleToWriteCallback(0, FORWARD_PO_KEY, forward_PO);
	redis_client.addDoubleToWriteCallback(0, RC_KEY, gain_scaling);
	redis_client.addDoubleToWriteCallback(0, CORRECTION_ENERGY_BACKWARD_KEY, E_correction_backwards);
	redis_client.addDoubleToWriteCallback(0, ENERGY_TO_DISSIPATE_BACKWARD_KEY, E_to_dissipate_backward);
	redis_client.addDoubleToWriteCallback(0, CORRECTION_ENERGY_FORWARD_KEY, E_correction_forward);
	redis_client.addDoubleToWriteCallback(0, DESIRED_EE_FORCE_KEY, Fd);
	redis_client.addDoubleToWriteCallback(0, SENSED_EE_FORCE_KEY, Fs);
	redis_client.addDoubleToWriteCallback(0, VC_KEY, vc);
	redis_client.addDoubleToWriteCallback(0, ALPHA_KEY, alpha_forward);

	redis_client.set(CONTROLLER_RUNNING_KEY, "1");

	// create a timer
	double dt = 0.001;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1/dt); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) {
	// wait for next scheduled loop
	timer.waitForNextLoop();
	double time = timer.elapsedTime() - start_time;

	// // read robot state from redis
	redis_client.executeReadCallback(0);
	robot->updateModel();

	vr = robot->_dq(0);
	Fs = sensed_force(2);

	if(state == MOVE_TO_CONTACT)
	{

		Fcmd = Fd - kv_force * vr;
		command_torques << Fcmd;

		if(olfc_counter <= 0)
		{
			t0 = time;
			state = FORCE_CONTROL;
		}
		olfc_counter--;
	}

	else if(state == FORCE_CONTROL)
	{
		// freq += freq_increment_per_second * dt;
		// if(freq > freq_final)
		// {
		// 	runloop = false;
		// }
		// Fd = amplitude * sin(2*M_PI*freq * (time - t0));

		integrated_force_error += (Fs - Fd) * dt;
		// if(abs(Fs - Fd) < 1e-2)
		// {
		// 	integrated_force_error = 0;
		// }
		if(abs(integrated_force_error) > 25 / ki_force)
		{
			integrated_force_error *= 25/abs(integrated_force_error)/ki_force;
		}
		vc = - kp_force * (Fs - Fd) - ki_force * integrated_force_error;
		double Fc = Rc * vc;
		Fcmd = k_ff	* Fd + vc - kv_force * vr;

		// E_stored = 0.5 * ki_force * integrated_force_error * integrated_force_error;

		// passivity observer
		// double power_1 = (Fd - Fs) * vc * dt;
		double power_1 = Fs * vc * dt;
		double power_5 = Fcmd * vr * dt;
		power_5 = 0;

		// if(power_1 < 0)
		// {
			backward_PO += power_1;	
		// }
		// else
		// {
			// forward_PO += power_1;
		// }
		// if(power_5 < 0)
		// {
		// 	backward_PO -= power_5;	
		// }
		// else
		// {
		// 	forward_PO -= power_5;
		// }

		// passivity controller backward path
		Rpc = 0;
		if(backward_PO + E_correction_backwards + E_stored < 0)
		{
			// E_to_dissipate_backward -= backward_PO + E_correction_backwards + E_stored;
		// }
			// Rc = Rc + (backward_PO + E_correction_backwards + E_stored)/(vc * vc * dt);
			// Rc = 1 + (backward_PO + E_correction_backwards + E_stored)/(vc * vc * dt);
			// Rc = 1 + abs(Fs - Fd) * (backward_PO + E_correction_backwards + E_stored + 0.1)/(vc * vc * dt);
			// Rc = 1 + abs(vc) * (backward_PO + E_correction_backwards + E_stored)/(vc * vc * dt);
			// Rpc = log(1-(backward_PO + E_correction_backwards + E_stored));
			Rpc = exp(-(backward_PO + E_correction_backwards + E_stored)) - 1;
			// Rpc = Rpc + exp(E_to_dissipate_backward) - 1;
			// Rpc = exp(E_to_dissipate_backward) - 1;
			// Rpc = - Rc * (backward_PO + E_correction_backwards + E_stored) / (Rc * vc * vc * dt);
		}
		if(Rpc < 0)
		{
			Rpc = 0;
		}
		gain_scaling = Rc/(Rc + Rpc);
		// // passivity controller forward path
		// alpha_forward = 0;
		// if(forward_PO + E_correction_forward < 0)
		// {
		// 	if(vr > 1e-2)
		// 	{
		// 		alpha_forward = - (forward_PO + E_correction_forward)/(vr * vr * dt);
		// 	}
		// 	if(alpha_forward > alpha_max)
		// 	{
		// 		alpha_forward = alpha_max;
		// 	}
		// }

		// energy correction
		double current_E_correction = Rpc * gain_scaling * vc * vc * dt;
		E_correction_backwards += current_E_correction;
		E_to_dissipate_backward -= current_E_correction;
		if(E_to_dissipate_backward < 0)
		{
			E_to_dissipate_backward = 0;
		}
		// E_correction_backwards += (1 - Rc) * vc * vc * dt;
		// E_correction_forward += (1 - Rc) * vc * vr * dt;
		// E_correction_forward += alpha_forward * vr * vr * dt;

		windowed_PO.push(power_1);
		windowed_E_correction.push(current_E_correction);

		if(windowed_PO.size() > windowed_PO_size)
		{
			backward_PO -= windowed_PO.front();
			E_correction_backwards -= windowed_E_correction.front();
			windowed_PO.pop();
			windowed_E_correction.pop();
		}

		// gain_scaling = 1;

		Fcmd = k_ff * Fd + Fc * gain_scaling - kv_force * vr - alpha_forward * vr;
		command_torques << Fcmd;
	}

	// // send to redis
	redis_client.executeWriteCallback(0);

	controller_counter++;

	}

	redis_client.set(CONTROLLER_RUNNING_KEY, "0");

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);


	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
