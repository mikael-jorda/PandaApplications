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

const string robot_file = "../resources/04-control_nullspace_collision/panda_arm.urdf";
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
const string KP_ORI_KEY = "sai2::PandaApplication::controller:kp_ori";
const string KV_ORI_KEY = "sai2::PandaApplication::controller:kv_ori";

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool inertia_regularization = true;

// logging
const string VEL_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::velocity_based_observer";
const string PROCESSED_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::processed_observer";
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

	auto joint_task = new Sai2Primitives::JointTask(robot);

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 15.0;
	// redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
	// redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));

	joint_task->_max_velocity = 45.0 * M_PI/180.0;

	VectorXd q_init_desired = VectorXd::Zero(dof);
	q_init_desired << 0, -45, 0, -125, 0, 130, 0;
	q_init_desired *= M_PI/180;
	joint_task->_goal_position = q_init_desired;

	// pos task
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.2);
	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);

	VectorXd pos_task_torques = VectorXd::Zero(dof);

	pos_task->_max_velocity = 0.05;


	pos_task->_kp = 200.0;
	pos_task->_kv = 23.0;
	// pos_task->_ki = 50.0;
	redis_client.set(KP_POS_KEY, to_string(pos_task->_kp));
	redis_client.set(KV_POS_KEY, to_string(pos_task->_kv));

	// prepare observers
	VectorXd gravity(dof), coriolis(dof);
	VectorXd r_vel = VectorXd::Zero(dof);
	VectorXd integral_vel = VectorXd::Zero(dof);

	MatrixXd K0 = 50*MatrixXd::Identity(dof,dof);
	bool first_iteration = true;
	double contact_detection_treshold = 1;
	double link_detection_treshold = 0.1;
	int link_in_contact = -1;
	VectorXd r_filtered = VectorXd::Zero(dof);
	bool in_contact = false;

	MatrixXd Lambda_contact = MatrixXd::Zero(1,1);
	MatrixXd Jbar_contact = MatrixXd::Zero(dof,1);
	MatrixXd Jbar_contact_np = MatrixXd::Zero(dof,1);
	MatrixXd J_contact = MatrixXd::Zero(1,dof);
	MatrixXd N_contact = MatrixXd::Identity(dof,dof);

	ButterworthFilter filter_r = ButterworthFilter(dof, 0.015);

	// cylinder model for the links
	MatrixXd cylinder_axis = MatrixXd::Zero(7, 3);
	MatrixXd cylinder_origin = MatrixXd::Zero(7,3);
	MatrixXd cylinder_shape = MatrixXd::Zero(7,2);
	cylinder_axis << 0 ,  0 , 1 , 
					 0 , -1 , 0 , 
					 0 ,  0 , 1 , 
					 0 ,  1 , 0 , 
					 0 ,  0 , 1 ,
					 0 ,  1 , 0 ,
					 0 ,  0 , 1 ;
	cylinder_origin <<  0    ,  0    , -0.2  ,
					    0    ,  0    ,  0    , 
					    0    ,  0    , -0.1  ,
					   -0.08 ,  0    ,  0    ,
					    0    ,  0    , -0.2  ,
					    0.08 , -0.05 ,  0    ,
					    0    ,  0    ,  0.07 ; 
	cylinder_shape << 0.06 , 0.2  ,
					  0.06 , 0.2  ,
					  0.06 , 0.1  ,
					  0.06 , 0.1  ,
					  0.06 , 0.2  ,
					  0.06 , 0.1  ,
					  0.08 , 0.03 ;	
	MatrixXd J_sample = MatrixXd::Zero(1,dof);
	double J_norm_estimate = 1;
	Vector3d sample_point = Vector3d::Zero();
	vector<string> link_names;
	link_names.push_back("link1");
	link_names.push_back("link2");
	link_names.push_back("link3");
	link_names.push_back("link4");
	link_names.push_back("link5");
	link_names.push_back("link6");
	link_names.push_back("link7");

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
			pos_task->_kp = stod(redis_client.get(KP_POS_KEY));
			pos_task->_kv = stod(redis_client.get(KV_POS_KEY));
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

		if(state == GO_TO_INIT_CONFIG)
		{
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			if((joint_task->_current_position - joint_task->_goal_position).norm() < 1e-3)
			{
				// joint_task->_kp = stod(redis_client.get(KP_JOINT_KEY));
				joint_task->_kp = 10;
				joint_task->_kv = 2*sqrt(joint_task->_kp);

				pos_task->reInitializeTask();
				pos_task->_goal_position(1) += 0.20;
				state_init_counter = controller_counter;
				state = HOLD_CARTESIAN_POS;
			}
		}

		else if(state == HOLD_CARTESIAN_POS)
		{
			// update tasks model
			N_prec.setIdentity();
			pos_task->updateTaskModel(N_prec);
			N_prec = pos_task->_N;

			if(in_contact)
			{
				robot->operationalSpaceMatrices(Lambda_contact, Jbar_contact_np, N_contact, J_contact, N_prec);
				J_contact = J_contact * N_prec;
				robot->operationalSpaceMatrices(Lambda_contact, Jbar_contact, N_contact, J_contact, N_prec);
				N_prec = pos_task->_N * N_contact;
			}

			joint_task->updateTaskModel(N_prec);

			// compute velocity based observer
			if(!first_iteration)
			{
				integral_vel += robot->_M_inv * (command_torques - coriolis + r_vel) * dt;
				r_vel = K0 * (robot->_dq - integral_vel);
			}

			// process velocity observer
			r_filtered = filter_r.update(r_vel);
			in_contact = false;
			link_in_contact = -1;
			if(r_filtered.norm() > contact_detection_treshold)
			{
				in_contact = true;
				for(int i=0 ; i<dof ; i++)
				{
					if(r_filtered(i) > link_detection_treshold || -r_filtered(i) > link_detection_treshold)
					{
						link_in_contact = i;						
					}
				}
				if(link_in_contact == -1)
				{
					throw std::runtime_error("detected contact but no link is in contact");
				}
				double random_theta = 2*M_PI*(rand()/RAND_MAX);
				double random_length = (rand()/RAND_MAX) * cylinder_shape(link_in_contact,1);
				double r = cylinder_shape(link_in_contact,0);
				sample_point.setZero();
				if(cylinder_axis(link_in_contact,2) != 0)
				{
					sample_point = Vector3d(r*cos(random_theta), r*sin(random_theta), cylinder_axis(link_in_contact,2)*random_length);
				}
				else if(cylinder_axis(link_in_contact,1) != 0)
				{
					sample_point = Vector3d(r*cos(random_theta), cylinder_axis(link_in_contact,1)*random_length, r*sin(random_theta));
				}
				else if(cylinder_axis(link_in_contact,0) != 0)
				{
					sample_point = Vector3d(cylinder_axis(link_in_contact,0)*random_length, r*cos(random_theta), r*sin(random_theta));
				}
				robot->Jv(J_sample, link_names[link_in_contact], sample_point);
				JacobiSVD<MatrixXd> svd(J_sample, ComputeThinU | ComputeThinV);
				J_norm_estimate = svd.singularValues()(0);
				// J_norm_estimate = 0.7;

				J_contact = r_filtered.transpose()/r_filtered.norm() * J_norm_estimate;
				// cout << J_contact << endl;
			}
			// MatrixXd Jv_contact = MatrixXd::Zero(3,dof);
			// robot->Jv(Jv_contact, "link5", Vector3d(0,0.06,-0.1));
			// // J_contact = Jv_contact.block(1,dof,1,0);
			// for(int i=0 ; i<dof ; i++)
			// {
			// 	J_contact(i) = Jv_contact(1,i);
			// }
			// J_contact = J_contact * pos_task->_N;
			// robot->operationalSpaceMatrices(Lambda_contact, Jbar_contact_np, N_contact, J_contact, N_prec);

			// compute torques
			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);

			contact_compensation_torques.setZero();
			if(in_contact)
			{
				double target_contact_force = 2;
				// double contact_force_estimate = (double)(Jbar_contact.transpose() * r_filtered)(0);
				// if(contact_force_estimate < target_contact_force)
				// {
					// target_contact_force = contact_force_estimate;
				// }
				// if(target_contact_force < 0)
				// {
					// target_contact_force = 0;
				// }
				contact_compensation_torques += pos_task->_projected_jacobian.transpose() * pos_task->_Jbar.transpose() * r_filtered;
				contact_compensation_torques += target_contact_force*J_contact.transpose() + 15*J_contact.transpose()*J_contact*robot->_dq;
			}

			command_torques = pos_task_torques + joint_task_torques + coriolis - contact_compensation_torques;
			// command_torques = pos_task_torques + joint_task_torques + coriolis;

			if(controller_counter - state_init_counter == 10000)
			{
				pos_task->_goal_position(1) -= 0.25;
			}

			if(first_iteration)
			{
				first_iteration = false;
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
		redis_client.setEigenMatrixJSON(PROCESSED_OBSERVER_LOGGING_KEY, r_filtered);
		redis_client.setEigenMatrixJSON(CONTACT_COMPENSATION_TORQUES_KEY, contact_compensation_torques);
		redis_client.setEigenMatrixJSON(COMMAND_TORQUES_LOGGING_KEY, command_torques);
		redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, pos_task->_desired_position);
		redis_client.setEigenMatrixJSON(CURRENT_POS_KEY, pos_task->_current_position);

		redis_client.setEigenMatrixJSON("test", r_filtered.transpose() * pos_task_torques);

		if(controller_counter % 50 == 0)
		{
			// cout << r_filtered.transpose() * (command_torques + contact_compensation_torques) << endl;
			cout << Jbar_contact.transpose() * r_filtered << endl;
			// cout << J_contact * robot->_dq << endl;
			// cout << "velocity based observer :\n" << r_vel.transpose() << endl;
			// cout << "processed observer :\n" << r_filtered.transpose() << endl;
			// cout << "position error :\n" << (pos_task->_current_position - pos_task->_desired_position).transpose() << endl;
			// cout << endl;
		}

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
