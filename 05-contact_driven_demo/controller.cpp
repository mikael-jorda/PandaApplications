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

const string robot_file = "./resources/panda_arm.urdf";
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

// const bool inertia_regularization = true;
const bool inertia_regularization = false;

// logging
const string MOM_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::momentum_based_observer";
const string PROCESSED_OBSERVER_LOGGING_KEY = "sai2::PandaApplication::controller::logging::processed_observer";
const string CONTACT_COMPENSATION_TORQUES_KEY = "sai2::PandaApplication::controller::logging::contact_compensation_torques";
const string DESIRED_POS_KEY = "sai2::PandaApplication::controller::logging::desired_position";
const string CURRENT_POS_KEY = "sai2::PandaApplication::controller::logging::current_position";
const string DESIRED_JOINT_KEY = "sai2::PandaApplication::controller::logging::desired_joint_position";
const string CURRENT_JOINT_KEY = "sai2::PandaApplication::controller::logging::current_joint_position";
const string LINK_IN_CONTACT_KEY = "sai2::PandaApplication::controller::logging::link_in_contact";
const string COMMAND_TORQUES_LOGGING_KEY = "sai2::PandaApplication::controller::logging::command_torques";
const string SENSED_TORQUES_LOGGING_KEY = "sai2::PandaApplication::controller::logging::sensed_torques";

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";
		JOINT_TORQUES_SENSED_KEY = "sai2::PandaApplication::actuators::fgc";

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
	// joint_task->_otg->setMaxVelocity(M_PI/8);
	// joint_task->_otg->setMaxAcceleration(M_PI/2);
	// joint_task->_otg->setMaxJerk(2*M_PI);

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd joint_task_torques_np = VectorXd::Zero(dof);
	joint_task->_kp = 35.0;
	joint_task->_kv = 10.0;
	joint_task->_ki = 50.0;
	redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
	redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));

	VectorXd q_init_desired = VectorXd::Zero(dof);
	q_init_desired << 0, 15, 0, -95, 0, 125, 0;
	q_init_desired *= M_PI/180;
	joint_task->_desired_position = q_init_desired;

	// pos task
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0.0,0.0,0.1);
	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);
	double max_vel = 0.1;
	pos_task->_otg->setMaxVelocity(max_vel);
	pos_task->_otg->setMaxAcceleration(4*max_vel);
	pos_task->_otg->setMaxJerk(16*max_vel);
	// pos_task->_use_interpolation_flag = false;
	// pos_task->_use_velocity_saturation_flag = true;
	// pos_task->_saturation_velocity = Vector3d(0.1,0.1,0.1);

	VectorXd pos_task_torques = VectorXd::Zero(dof);

	pos_task->_kp = 300.0;
	pos_task->_kv = 25.0;
	// pos_task->_ki = 50.0;
	redis_client.set(KP_POS_KEY, to_string(pos_task->_kp));
	redis_client.set(KV_POS_KEY, to_string(pos_task->_kv));

	// prepare observers
	VectorXd gravity(dof), coriolis(dof);
	VectorXd r_mom = VectorXd::Zero(dof);
	VectorXd p = VectorXd::Zero(dof);
	VectorXd beta = VectorXd::Zero(dof);
	MatrixXd M_prev = MatrixXd::Zero(dof,dof);
	MatrixXd M_dot = MatrixXd::Zero(dof,dof);
	VectorXd integral_mom = VectorXd::Zero(dof);

	VectorXd K = VectorXd::Zero(dof);
	// K << 50, 50, 50, 50, 50, 50, 50;
	K << 25, 25, 25, 25, 25, 25, 25;
	// K << 1, 1, 1, 1, 1, 1, 1;
	MatrixXd K0 = MatrixXd::Identity(dof,dof);
	for(int i=0 ; i<dof ; i++)
	{
		K0(i,i) = K(i);
	}
	bool first_iteration = true;
	// double link_detection_treshold = 2.5;
	double link_detection_treshold = 0.3;
	int link_in_contact = -1;
	VectorXd r_filtered = VectorXd::Zero(dof);
	bool in_contact = false;
	bool in_contact_prev = false;

	MatrixXd Lambda_contact = MatrixXd::Zero(1,1);
	MatrixXd Jbar_contact = MatrixXd::Zero(dof,1);
	MatrixXd Jbar_contact_np = MatrixXd::Zero(dof,1);
	MatrixXd Proj_contact = MatrixXd::Zero(dof,dof);
	MatrixXd J_contact = MatrixXd::Zero(1,dof);
	MatrixXd J_contact_control = MatrixXd::Zero(1,dof);
	MatrixXd N_contact = MatrixXd::Identity(dof,dof);

	ButterworthFilter filter_r = ButterworthFilter(dof, 0.015);

	// link centers
	MatrixXd J_sample = MatrixXd::Zero(3,dof);
	MatrixXd Lambda_sample = MatrixXd::Zero(3,3);
	MatrixXd N_sample = MatrixXd::Zero(dof,dof);
	MatrixXd Jbar_sample = MatrixXd::Zero(dof,3);
	double J_norm_estimate = 1;

	vector<string> link_names;
	link_names.push_back("link1");
	link_names.push_back("link2");
	link_names.push_back("link3");
	link_names.push_back("link4");
	link_names.push_back("link5");
	link_names.push_back("link6");
	link_names.push_back("link7");

	vector<Vector3d> center_points;
	center_points.push_back(Vector3d(0.0, 0.0, -0.1));
	center_points.push_back(Vector3d(0.0, -0.07, 0.0));
	center_points.push_back(Vector3d(0.0, 0.0, -0.07));
	center_points.push_back(Vector3d(-0.088, 0.05, 0.0));
	center_points.push_back(Vector3d(0.0, 0.0, -0.1));
	center_points.push_back(Vector3d(0.088, 0.0, 0.0));
	center_points.push_back(Vector3d(0.0, 0.0, 0.075));

	int r_buffer_length = 20;
	MatrixXd r_buffer = MatrixXd::Zero(dof,r_buffer_length);
	VectorXd r_norm_buffer = VectorXd::Zero(r_buffer_length);
	VectorXd r_link_in_contact_buffer = -1*VectorXd::Ones(r_buffer_length);
	VectorXd r_link_in_contact_count = VectorXd::Zero(dof);
	VectorXd r_mean = VectorXd::Zero(dof);

	// torques to compensate contact at task level
	VectorXd contact_compensation_torques = VectorXd::Zero(dof);
	VectorXd contact_force_control_torques = VectorXd::Zero(dof);
	double direction = 0;

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
		M_prev = robot->_M;
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
		M_dot = (robot->_M - M_prev)/dt;


		if(state == GO_TO_INIT_CONFIG)
		{
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			if((joint_task->_current_position - joint_task->_desired_position).norm() < 0.2)
			{
				joint_task->_ki = 0.0;

				pos_task->reInitializeTask();
				pos_task->_desired_position(1) += 0.2;
				state_init_counter = controller_counter;
				state = HOLD_CARTESIAN_POS;
			}
		}

		else if(state == HOLD_CARTESIAN_POS)
		{
			// if(controller_counter % 20 == 0)
			{
				// update tasks model
				N_prec.setIdentity();
				pos_task->updateTaskModel(N_prec);
				N_prec = pos_task->_N;

				// for direction monitoring
				joint_task->updateTaskModel(N_prec);
				joint_task->computeTorques(joint_task_torques_np);

				if(in_contact)
				{
					Eigen::VectorXd Fc_est = VectorXd::Zero(3);
					if(link_in_contact == 0)
					{
						J_norm_estimate = 1;
						J_contact_control = r_filtered.transpose()/r_filtered.norm() * J_norm_estimate;
					}
					else
					{
						robot->Jv(J_sample, link_names[link_in_contact], center_points[link_in_contact]);
						JacobiSVD<MatrixXd> svd(J_sample, ComputeFullU | ComputeFullV);
						J_norm_estimate = svd.singularValues()(0);

						Eigen::MatrixXd J_inv = MatrixXd::Zero(dof,3);
						Eigen::MatrixXd Sigma_inv = MatrixXd::Zero(dof, 3);
						for(int i=0 ; i<3 ; i++)
						{
							if(svd.singularValues()(i) > 1e-3)
							{
								Sigma_inv(i,i) = 1/svd.singularValues()(i);
							}
						}
						J_inv = svd.matrixV() * Sigma_inv * svd.matrixU().transpose();
						Fc_est = J_inv.transpose()*r_mom;
						J_contact_control = Fc_est.transpose()*J_sample/Fc_est.norm();

					}

					J_contact_control = J_contact_control * N_prec;

					J_contact = r_filtered.transpose()/r_filtered.norm();
					robot->operationalSpaceMatrices(Lambda_contact, Jbar_contact_np, N_contact, J_contact, N_prec);
					J_contact = J_contact * N_prec;
					robot->operationalSpaceMatrices(Lambda_contact, Jbar_contact, N_contact, J_contact, N_prec);
					Proj_contact = Jbar_contact*J_contact;
					N_prec = N_prec * N_contact;
				

					joint_task->updateTaskModel(N_prec);
				}

			}

			// compute momentum based observer
			if(!first_iteration)
			{
				p = robot->_M * robot->_dq;
				beta = coriolis - M_dot * robot->_dq;
				// beta = coriolis;
				integral_mom += (command_torques - beta + r_mom) * dt;
				r_mom = K0 * (p - integral_mom);
			}

			// process velocity observer
			r_filtered = filter_r.update(r_mom);
			int i_buffer = controller_counter % r_buffer_length;
			r_norm_buffer(i_buffer) = r_filtered.norm();
			in_contact_prev = in_contact;
			if(r_link_in_contact_buffer[i_buffer] != -1)
			{
				r_link_in_contact_count(r_link_in_contact_buffer[i_buffer]) -= 1;
			}
			link_in_contact = -1;
			for(int i=0 ; i<dof ; i++)
			{
				if(r_filtered(i) > link_detection_treshold || -r_filtered(i) > link_detection_treshold)
				{
					link_in_contact = i;						
				}
			}
			r_link_in_contact_buffer(i_buffer) = link_in_contact;
			if(link_in_contact > -1)
			{
				r_link_in_contact_count(link_in_contact) += 1;
			}

			bool buffer_all_true = true;
			bool buffer_all_false = true;

			for (int i = 0; i < r_buffer_length; i++)
			{
				buffer_all_true = buffer_all_true && (r_link_in_contact_buffer(i) != -1);
				buffer_all_false = buffer_all_false && (r_link_in_contact_buffer(i) == -1);
			}

			if(buffer_all_true)
			{
				in_contact = true;
			}
			else if(buffer_all_false)
			{
				in_contact = false;
			}
			else
			{
				in_contact = in_contact_prev;
			}

			if(in_contact)
			{
				int lc = 0;
				for(int i=0 ; i<dof ; i++)
				{
					if(r_link_in_contact_count(i) > r_link_in_contact_count(lc))
					{
						lc = i;
					}
				}
				link_in_contact = lc;
			}

			if(!in_contact && in_contact_prev)
			{
				cout << "lost contact" << endl;
				joint_task->reInitializeTask();
				joint_task->_desired_position = q_init_desired;
			}

			// compute torques
			pos_task->computeTorques(pos_task_torques);
			joint_task->computeTorques(joint_task_torques);

			// contact_compensation_torques.setZero(dof);
			contact_compensation_torques = pos_task->_projected_jacobian.transpose() * pos_task->_Jbar.transpose() * r_mom;
			contact_force_control_torques.setZero(dof);
			if(in_contact)
			{
				double target_contact_force = 0;
				direction = (double) (Jbar_contact_np.transpose() * (pos_task_torques + joint_task_torques_np))(0);
				if(direction < 0)
				{
					target_contact_force = 10;
				}
				contact_force_control_torques = target_contact_force*J_contact_control.transpose();
			}

			command_torques = pos_task_torques + joint_task_torques + coriolis - contact_compensation_torques - contact_force_control_torques;
			// command_torques = pos_task_torques + joint_task_torques + coriolis - contact_compensation_torques;
			// command_torques = pos_task_torques + joint_task_torques + coriolis;

			if((controller_counter - state_init_counter) % 20000 == 5000)
			{
				pos_task->_desired_position(1) -= 0.2;
			}
			else if((controller_counter - state_init_counter) % 20000 == 15000)
			{
				pos_task->_desired_position(1) += 0.2;
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
		VectorXd torques_sensed = redis_client.getEigenMatrixJSON(JOINT_TORQUES_SENSED_KEY);

		redis_client.setEigenMatrixJSON(MOM_OBSERVER_LOGGING_KEY, r_mom);
		redis_client.setEigenMatrixJSON(PROCESSED_OBSERVER_LOGGING_KEY, r_filtered);
		redis_client.setEigenMatrixJSON(CONTACT_COMPENSATION_TORQUES_KEY, contact_compensation_torques);
		redis_client.setEigenMatrixJSON(COMMAND_TORQUES_LOGGING_KEY, command_torques);
		redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, pos_task->_desired_position);
		redis_client.setEigenMatrixJSON(CURRENT_POS_KEY, pos_task->_current_position);
		redis_client.setEigenMatrixJSON(DESIRED_JOINT_KEY, joint_task->_desired_position);
		redis_client.setEigenMatrixJSON(CURRENT_JOINT_KEY, joint_task->_current_position);
		redis_client.setEigenMatrixJSON(SENSED_TORQUES_LOGGING_KEY, torques_sensed);
		redis_client.set(LINK_IN_CONTACT_KEY, to_string(link_in_contact));

		if(controller_counter % 500 == 0)
		{
			// cout << in_contact << endl;
			// cout << r_filtered.transpose() * (command_torques + contact_compensation_torques) << endl;
			cout << "filtered observer :\n" << r_filtered.transpose() << endl;
			cout << "link in contact buffer :\n" << r_link_in_contact_buffer.transpose() << endl;
			cout << "link in contact count :\n" << r_link_in_contact_count.transpose() << endl;
			cout << "link in contact : " << link_in_contact << endl;
			cout << "direction : " << Jbar_contact_np.transpose() * (pos_task_torques + joint_task_torques_np) << endl;
			// cout << "J contact control : " << J_contact_control << endl;
			// cout << J_contact * robot->_dq << endl;
			// cout << "velocity based observer :\n" << r_vel.transpose() << endl;
			// cout << "norm of r_vel : " << r_filtered.norm() << endl;
			// cout << "sensed torques :\n" << (filtered_sensed_torques - gravity).transpose() << endl; 
			// cout << "link in contact : " << link_in_contact << endl;
			// cout << "norm J : " << J_norm_estimate << endl;
			// cout << "position error :\n" << (pos_task->_current_position - pos_task->_desired_position).transpose() << endl;
			cout << endl;
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
