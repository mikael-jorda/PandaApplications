// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"

#include <iostream>
#include <string>
#include <queue>

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_spring.urdf";
const string robot_file = "./resources/pbot.urdf";
const string robot_name = "PBOT";

#define MOVE_TO_CONTACT      0
#define FORCE_CONTROL        1

int state = MOVE_TO_CONTACT;

// logging
const string DESIRED_EE_FORCE_KEY = "sai2::PandaApplication::controller:desried_ee_force";
const string SENSED_EE_FORCE_KEY = "sai2::PandaApplication::controller:sensed_ee_force";
const string RC_KEY = "sai2::PandaApplication::controller:Rc";
const string FORWARD_PO_KEY = "sai2::PandaApplication::controller:forward_PO";
const string BACKWARD_PO_KEY = "sai2::PandaApplication::controller:backward_PO";
const string STORED_ENERGY_KEY = "sai2::PandaApplication::controller:E_stored";
const string CORRECTION_ENERGY_BACKWARD_KEY = "sai2::PandaApplication::controller:E_correction_backward";
const string CORRECTION_ENERGY_FORWARD_KEY = "sai2::PandaApplication::controller:E_correction_forward";
const string VC_KEY = "sai2::PandaApplication::controller:vc";

const string CONTROLLER_RUNNING_KEY = "sai2::PandaApplication::controller::flag_running";
RedisClient redis_client;

VectorXd command_torques;
Vector3d delayed_sensed_force;

unsigned long long controller_counter = 0;
int olfc_counter = 0;

// controller function prototype
void control(Sai2Model::Sai2Model* robot);

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->updateKinematics();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.6);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateKinematics();

	// prepare simulation
	int dof = robot->dof();
	command_torques = VectorXd::Zero(robot->dof());
	Vector3d sensed_force = Vector3d::Zero();

	// force from environment
	double k_env = 150.0;
	double b_env = 10.0;
	double x_env = 0.0;

	double Fenv = 0;

	const int introduced_delay = 2; // timesteps
	queue<Vector3d> force_buffer;
	delayed_sensed_force = Vector3d::Zero();

	// start control thread
	runloop = true;
	thread control_thread(control, robot);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(2000); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (runloop) {
		fTimerDidSleep = timer.waitForNextLoop();

		// compute environment force
		Fenv = - k_env * (robot->_q(0) - x_env) - b_env * robot->_dq(0);
		command_torques(0) += Fenv;

		// set torques to simulation
		sim->setJointTorques(robot_name, command_torques);

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// // update force sensor and read values
		// fsensor->update(sim);
		// fsensor->getForceLocalFrame(sensed_force);
		sensed_force = Vector3d(0,0,Fenv);
		// force_buffer[buffer_counter] = Vector3d(0,0,Fenv);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();

		// write new robot state to redis
		// redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
		// redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
		if(introduced_delay > 0)
		{
			force_buffer.push(-sensed_force);
			if(force_buffer.size() > introduced_delay)
			{
				delayed_sensed_force = force_buffer.front();
				force_buffer.pop();
			}
		}
		else
		{
			delayed_sensed_force = -sensed_force;
		}

		// if(simulation_counter % 100 == 0)
		// {
		// 	cout << robot->_q(0) << endl;
		// }

		//update last time
		last_time = curr_time;

		simulation_counter++;
	}

	runloop = false;
	control_thread.join();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

void control(Sai2Model::Sai2Model* robot)
{
	// prepare controller
	double Fd = 0;
	double Fcmd = 0;
	double vr = 0;
	double kp_force = 250.0;
	double ki_force = 150.0;
	double kv_force = 25.0;
	double k_ff = 1.0;

	double Fs = 0;

	double integrated_force_error = 0;

	// passivity
	double backward_PO = 0;
	double forward_PO = 0;
	double E_stored = 0;
	double E_correction_backwards = 0;
	double E_correction_forward = 0;

	double vc = 0;
	double Rc = 1.0;

	// force chirp
	double freq_init = 0.0;
	double freq_final = 10.0;
	double freq = freq_init;
	double freq_increment_per_second = 0.5;
	double amplitude = 0.5;

	double t0 = 0;

	// setup redis exchange
	redis_client.createWriteCallback(0);

	redis_client.addDoubleToWriteCallback(0, BACKWARD_PO_KEY, backward_PO);
	redis_client.addDoubleToWriteCallback(0, FORWARD_PO_KEY, forward_PO);
	redis_client.addDoubleToWriteCallback(0, RC_KEY, Rc);
	redis_client.addDoubleToWriteCallback(0, CORRECTION_ENERGY_BACKWARD_KEY, E_correction_backwards);
	redis_client.addDoubleToWriteCallback(0, CORRECTION_ENERGY_FORWARD_KEY, E_correction_forward);
	redis_client.addDoubleToWriteCallback(0, DESIRED_EE_FORCE_KEY, Fd);
	redis_client.addDoubleToWriteCallback(0, SENSED_EE_FORCE_KEY, Fs);
	redis_client.addDoubleToWriteCallback(0, VC_KEY, vc);

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
	robot->updateModel();

	vr = robot->_dq(0);
	Fs = delayed_sensed_force(2);

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
		freq += freq_increment_per_second * dt;
		if(freq > freq_final)
		{
			runloop = false;
		}
		Fd = amplitude * sin(2*M_PI*freq * (time - t0));

		integrated_force_error += (Fs - Fd) * dt;
		vc = - kp_force * (Fs - Fd) - ki_force * integrated_force_error;
		Fcmd = k_ff	* Fd + vc - kv_force * vr;

		// passivity observer
		double power_1 = (Fs - Fd) * vc * dt;
		double power_5 = Fcmd * vr * dt;

		if(power_1 < 0)
		{
			backward_PO += power_1;	
		}
		else
		{
			forward_PO += power_1;
		}
		if(power_5 < 0)
		{
			backward_PO -= power_5;	
		}
		else
		{
			forward_PO -= power_5;
		}

		// // passivity controller
		// E_correction_backwards += (1 - Rc) * vc * vc * dt;

		// Rc = 1;
		// if(backward_PO + E_correction_backwards < 0)
		// {
		// 	Rc = 1 + (backward_PO + E_correction_backwards)/(vc * vc * dt);
		// }
		// if(Rc < 0)
		// {
		// 	Rc = 0;
		// }


		Fcmd = k_ff * Fd + vc * Rc - kv_force * vr;
		command_torques << Fcmd;
	}

	// send to redis
	redis_client.executeWriteCallback(0);

	controller_counter++;

	}

	redis_client.set(CONTROLLER_RUNNING_KEY, "0");
	command_torques.setZero();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}

