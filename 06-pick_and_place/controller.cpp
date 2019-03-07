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
const vector<string> JOINT_ANGLES_KEYS  = {
	"sai2::WarehouseSimulation::panda1::sensors::q",
	"sai2::WarehouseSimulation::panda2::sensors::q",
};
const vector<string> JOINT_VELOCITIES_KEYS = {
	"sai2::WarehouseSimulation::panda1::sensors::dq",
	"sai2::WarehouseSimulation::panda2::sensors::dq",
};

// - write
const vector<string> TORQUES_COMMANDED_KEYS = {
	"sai2::WarehouseSimulation::panda1::actuators::fgc",
	"sai2::WarehouseSimulation::panda2::actuators::fgc",
};

// - gripper
const vector<string> GRIPPER_MODE_KEYS = {   // m for move and g for graps
	"sai2::WarehouseSimulation::panda1::gripper::mode",
	"sai2::WarehouseSimulation::panda2::gripper::mode",
};
const vector<string> GRIPPER_CURRENT_WIDTH_KEYS = {
	"sai2::WarehouseSimulation::panda1::gripper::current_width",
	"sai2::WarehouseSimulation::panda2::gripper::current_width",
};
const vector<string> GRIPPER_DESIRED_WIDTH_KEYS = {
	"sai2::WarehouseSimulation::panda1::gripper::desired_width",
	"sai2::WarehouseSimulation::panda2::gripper::desired_width",
};
const vector<string> GRIPPER_DESIRED_SPEED_KEYS = {
	"sai2::WarehouseSimulation::panda1::gripper::desired_speed",
	"sai2::WarehouseSimulation::panda2::gripper::desired_speed",
};
const vector<string> GRIPPER_DESIRED_FORCE_KEYS = {
	"sai2::WarehouseSimulation::panda1::gripper::desired_force",
	"sai2::WarehouseSimulation::panda2::gripper::desired_force",
};

#define GO_TO_INIT_CONFIG       0
#define PICK_OBJECT             1
#define TRANSFER_OBJECT         2
#define PLACE_OBJECT            3
#define PRE_FINAL               4
#define FINAL            		5

int translation_counter = 0;

int state = GO_TO_INIT_CONFIG;
unsigned long long controller_counter = 0;

RedisClient redis_client;

int main() {

	// object gravity
	VectorXd object_gravity = VectorXd::Zero(6);
	object_gravity << 0, 0, -9.81, 0, 0, 0;

	// position of robots in world
	vector<Affine3d> robot_pose_in_world;
	Affine3d pose = Affine3d::Identity();
	pose.translation() = Vector3d(-0.5,-0.5,0);
	pose.linear() = AngleAxisd(0.3, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	pose.translation() = Vector3d(-0.5,0.5,0);
	pose.linear() = AngleAxisd(-0.3, Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	vector<Sai2Model::Sai2Model*> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(new Sai2Model::Sai2Model(robot_files[i], false));
		robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
		robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
		robots[i]->updateModel();
	}

	// prepare task controllers
	vector<int> dof;
	vector<VectorXd> command_torques;
	vector<VectorXd> coriolis;
	vector<MatrixXd> N_prec;

	vector<Sai2Primitives::JointTask*> joint_tasks;
	vector<VectorXd> joint_task_torques;
	vector<Sai2Primitives::PosOriTask*> posori_tasks;
	vector<VectorXd> posori_task_torques;

	for(int i=0 ; i<n_robots ; i++)
	{
		dof.push_back(robots[i]->dof());
		command_torques.push_back(VectorXd::Zero(dof[i]));
		coriolis.push_back(VectorXd::Zero(dof[i]));
		N_prec.push_back(MatrixXd::Identity(dof[i],dof[i]));

		// joint tasks
		joint_tasks.push_back(new Sai2Primitives::JointTask(robots[i]));
		joint_task_torques.push_back(VectorXd::Zero(dof[i]));

		joint_tasks[i]->_kp = 50.0;
		joint_tasks[i]->_kv = 14.0;

		VectorXd q_init_desired = VectorXd::Zero(dof[i]);
		q_init_desired << 0, 15, 0, -95, 0, 125, 0;
		joint_tasks[i]->_desired_position = M_PI/180*q_init_desired;

		// end effector tasks
		string link_name = "link7";
		Eigen::Vector3d pos_in_link = Vector3d(0.0,0.0,0.2);
		posori_tasks.push_back(new Sai2Primitives::PosOriTask(robots[i], link_name, pos_in_link));
		posori_task_torques.push_back(VectorXd::Zero(dof[i]));
		posori_tasks[i]->_use_interpolation_flag = false;

		posori_tasks[i]->_kp_pos = 200.0;
		posori_tasks[i]->_kv_pos = 25.0;
		posori_tasks[i]->_kp_ori = 400.0;
		posori_tasks[i]->_kv_ori = 40.0;		

		posori_tasks[i]->_use_velocity_saturation_flag = true;
		posori_tasks[i]->_linear_saturation_velocity = 0.1;
		posori_tasks[i]->_angular_saturation_velocity = 60.0/180.0;

	}
	// posori_tasks[1]->_angular_saturation_velocity = 20.0/180.0*M_PI*Vector3d::Ones();

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

		// read robot state from redis and update robot model
		for(int i=0 ; i<n_robots ; i++)
		{
			robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
			robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);

			robots[i]->updateModel();
			robots[i]->coriolisForce(coriolis[i]);
		}

		if(state == GO_TO_INIT_CONFIG)
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				joint_tasks[i]->updateTaskModel(N_prec[i]);

				joint_tasks[i]->computeTorques(joint_task_torques[i]);
				command_torques[i] = joint_task_torques[i] + coriolis[i];
			}

			double cumulative_joint_error = 0;
			for(int i=0 ; i<n_robots ; i++)
			{
				cumulative_joint_error += (joint_tasks[i]->_current_position - joint_tasks[i]->_desired_position).norm();
			}

			if(cumulative_joint_error < 0.3)
			{
				for(int i=0 ; i<n_robots ; i++)
				{
					posori_tasks[i]->reInitializeTask();
					redis_client.set(GRIPPER_DESIRED_WIDTH_KEYS[0], to_string(0.055));
					redis_client.set(GRIPPER_DESIRED_WIDTH_KEYS[1], to_string(0.045));
					state = PICK_OBJECT;
				}
			}
		}

		else if(state == PICK_OBJECT)
		{

			// update tasks model
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				posori_tasks[i]->updateTaskModel(N_prec[i]);

				N_prec[i] = posori_tasks[i]->_N;
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}

			// set goal positions
			posori_tasks[0]->_desired_position = robot_pose_in_world[0].linear().transpose()*(Vector3d(-0.07,-0.75,0.24) - robot_pose_in_world[0].translation());
			posori_tasks[0]->_desired_orientation = robot_pose_in_world[0].linear().transpose()*AngleAxisd(180.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(-45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();

			// compute torques
			for(int i=0 ; i<n_robots ; i++)
			{
				posori_tasks[i]->computeTorques(posori_task_torques[i]);
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i];
			}

			if((posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() < 1e-3)
			{
				translation_counter++;
			}
			else
			{
				translation_counter = 0;
			}
			if(translation_counter >= 250)
			{
				translation_counter = 0;
				redis_client.set(GRIPPER_MODE_KEYS[0], "g");
				redis_client.set(GRIPPER_DESIRED_FORCE_KEYS[0], to_string(35));
				state = TRANSFER_OBJECT;
			}

		}

		else if(state == TRANSFER_OBJECT)
		{
			// update tasks model
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				posori_tasks[i]->updateTaskModel(N_prec[i]);

				N_prec[i] = posori_tasks[i]->_N;
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}

			// set goal positions
			posori_tasks[0]->_desired_position = robot_pose_in_world[0].linear().transpose()*(Vector3d(-0.0,-0.025,0.5) - robot_pose_in_world[0].translation());
			posori_tasks[0]->_desired_orientation = robot_pose_in_world[0].linear().transpose()*AngleAxisd(-90.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();
			
			posori_tasks[1]->_desired_position = robot_pose_in_world[1].linear().transpose()*(Vector3d(-0.0,0.025,0.495) - robot_pose_in_world[1].translation());
			posori_tasks[1]->_desired_orientation = robot_pose_in_world[1].linear().transpose()*AngleAxisd(90.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(-45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();

			// compute torques
			for(int i=0 ; i<n_robots ; i++)
			{
				posori_tasks[i]->computeTorques(posori_task_torques[i]);
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i];
			}
			command_torques[0] -= posori_tasks[0]->_projected_jacobian.transpose()*object_gravity;

			if((posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() < 1e-2
				&& (posori_tasks[1]->_desired_position - posori_tasks[1]->_current_position).norm() < 1e-2)
			{
				translation_counter++;
			}
			else
			{
				translation_counter = 0;
			}
			if(translation_counter >= 250)
			{
				translation_counter = 0;
				redis_client.set(GRIPPER_MODE_KEYS[1], "g");
				redis_client.set(GRIPPER_DESIRED_FORCE_KEYS[1], to_string(55));
				redis_client.set(GRIPPER_MODE_KEYS[0], "m");
				redis_client.set(GRIPPER_DESIRED_WIDTH_KEYS[0], to_string(0.035));
				state = PLACE_OBJECT;
			}
		}

		else if(state == PLACE_OBJECT)
		{
			// update tasks model
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				posori_tasks[i]->updateTaskModel(N_prec[i]);

				N_prec[i] = posori_tasks[i]->_N;
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}

			// set goal positions
			posori_tasks[0]->_desired_position = robot_pose_in_world[0].linear().transpose()*(Vector3d(-0.0,-0.2,0.5) - robot_pose_in_world[0].translation());
			posori_tasks[0]->_desired_orientation = robot_pose_in_world[0].linear().transpose()*AngleAxisd(-90.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();
			
			posori_tasks[1]->_desired_position = robot_pose_in_world[1].linear().transpose()*(Vector3d(-0.05,0.7,0.27) - robot_pose_in_world[1].translation());
			posori_tasks[1]->_desired_orientation = robot_pose_in_world[1].linear().transpose()*AngleAxisd(180.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(-45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();
			// posori_tasks[1]->_desired_position = robot_pose_in_world[1].linear().transpose()*(Vector3d(-0.05,0.7,0.4) - robot_pose_in_world[1].translation());
			// posori_tasks[1]->_desired_orientation = robot_pose_in_world[1].linear().transpose()*AngleAxisd(90.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(-45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();

			// compute torques
			for(int i=0 ; i<n_robots ; i++)
			{
				posori_tasks[i]->computeTorques(posori_task_torques[i]);
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i];
			}
			command_torques[1] -= posori_tasks[1]->_projected_jacobian.transpose()*object_gravity;

			if((posori_tasks[0]->_desired_position - posori_tasks[0]->_current_position).norm() < 1e-2
				&& (posori_tasks[1]->_desired_position - posori_tasks[1]->_current_position).norm() < 1e-2)
			{
				translation_counter++;
			}
			else
			{
				translation_counter = 0;
			}
			if(translation_counter >= 100)
			{
				translation_counter = 0;
				state = PRE_FINAL;
			}

		}

		else if(state == PRE_FINAL)
		{
			// update tasks model
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				posori_tasks[i]->updateTaskModel(N_prec[i]);

				N_prec[i] = posori_tasks[i]->_N;
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}

			// set goal positions
			posori_tasks[0]->_desired_orientation = robot_pose_in_world[0].linear().transpose()*AngleAxisd(180.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix();
			
			posori_tasks[1]->_desired_position = robot_pose_in_world[1].linear().transpose()*(Vector3d(-0.05,0.7,0.2535) - robot_pose_in_world[1].translation());
			posori_tasks[1]->_desired_orientation = robot_pose_in_world[1].linear().transpose()*AngleAxisd(180.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(-45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();

			// compute torques
			for(int i=0 ; i<n_robots ; i++)
			{
				posori_tasks[i]->computeTorques(posori_task_torques[i]);
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i];
			}
			command_torques[1] -= posori_tasks[1]->_projected_jacobian.transpose()*object_gravity;

			if((posori_tasks[1]->_desired_position - posori_tasks[1]->_current_position).norm() < 1e-2)
			{
				translation_counter++;
			}
			else
			{
				translation_counter = 0;
			}
			if(translation_counter >= 100)
			{
				translation_counter = 0;
				redis_client.set(GRIPPER_MODE_KEYS[1], "m");
				redis_client.set(GRIPPER_DESIRED_WIDTH_KEYS[1], to_string(0.04));
				state = FINAL;
			}

		}

		else if(state == FINAL)
		{
			// update tasks model
			for(int i=0 ; i<n_robots ; i++)
			{
				N_prec[i].setIdentity();
				posori_tasks[i]->updateTaskModel(N_prec[i]);

				N_prec[i] = posori_tasks[i]->_N;
				joint_tasks[i]->updateTaskModel(N_prec[i]);
			}

			// set goal positions
			posori_tasks[1]->_desired_position = robot_pose_in_world[1].linear().transpose()*(Vector3d(-0.0,0.2,0.5) - robot_pose_in_world[1].translation());
			// posori_tasks[1]->_desired_orientation = robot_pose_in_world[1].linear().transpose()*AngleAxisd(180.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix()*AngleAxisd(-45.0/180.0*M_PI, Vector3d::UnitZ()).toRotationMatrix();
			posori_tasks[1]->_desired_orientation = robot_pose_in_world[1].linear().transpose()*AngleAxisd(180.0/180.0*M_PI, Vector3d::UnitX()).toRotationMatrix();

			// compute torques
			for(int i=0 ; i<n_robots ; i++)
			{
				posori_tasks[i]->computeTorques(posori_task_torques[i]);
				joint_tasks[i]->computeTorques(joint_task_torques[i]);

				command_torques[i] = posori_task_torques[i] + joint_task_torques[i] + coriolis[i];
			}

		}

		else
		{
			for(int i=0 ; i<n_robots ; i++)
			{
				command_torques[i].setZero(dof[i]);
			}
		}

		// send to redis
		for(int i=0 ; i<n_robots ; i++)
		{
			redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEYS[i], command_torques[i]);
		}

		prev_time = current_time;
		controller_counter++;
	}

	for(int i=0 ; i<n_robots ; i++)
	{
		command_torques[i].setZero();
		redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEYS[i], command_torques[i]);
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}