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
#define ALIGNMENT_PHASE      1
#define CLEAN_WINDOW         2
#define SWITCH_TOOL          3
#define DRY_WINDOW           4
#define END1                 5
#define END2                 6

int state = MOVE_TO_INITIAL;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;

std::string FORCE_SENSED_KEY; 

// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

const bool flag_simulation = false;
// const bool flag_simulation = true;

const bool inertia_regularization = false;

unsigned long long controller_counter = 0;

int main() {

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

		FORCE_SENSED_KEY = "sai2::PandaApplication::sensors::force_moment";

	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Bonnie::sensors::torques";

		FORCE_SENSED_KEY = "sai2::ATIGamma_Sensor::Bonnie::force_torque";

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
	Affine3d robot_pose_in_world;
	robot_pose_in_world.translation() = Vector3d(-0.06, 0.57, 0.0);
	robot_pose_in_world.linear() = AngleAxisd(-1.0864675, Vector3d::UnitZ()).toRotationMatrix();
	// auto robot = new Sai2Model::Sai2Model(robot_file, false, robot_pose_in_world);
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->updateModel();

	Matrix3d R_robot_world = robot_pose_in_world.linear().transpose();

	// prepare controller	
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	
	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	joint_task->_kp = 250.0;
	joint_task->_kv = 25.0;
	joint_task->_ki = 55.0;

	VectorXd max_q(dof);
	VectorXd min_q(dof);
	max_q << 2.8, 1.7, 2.85, -0.1, 2.8, 3.7, 2.9;
	min_q << -2.8, -1.7, -2.85, -3.0, -2.8, 0.05, -2.9;

	Eigen::VectorXd initial_q(dof);
	initial_q << -2.17553,-0.77296,1.04002,-1.48625,-0.0526487,2.61592,-0.795978;
	joint_task->_desired_position = initial_q;

	VectorXd target_q = (max_q + min_q + 5*initial_q)/7.0;

	// posori controller
	const string link_name = "link7";
	const Eigen::Vector3d pos_in_link = Vector3d(0,0,0.36);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(dof);

	posori_task->_kp_pos = 250.0;
	posori_task->_kv_pos = 22.0;
	posori_task->_kp_ori = 350.0;
	posori_task->_kv_ori = 27.0;

	posori_task->_kp_moment = 1.5;
	posori_task->_ki_moment = 2.5;
	posori_task->_kv_moment = 7.0;

	posori_task->_otg->setMaxLinearVelocity(0.12);
	posori_task->_otg->setMaxLinearAcceleration(0.2);
	posori_task->_otg->setMaxLinearJerk(1.0);

	posori_task->_otg->setMaxAngularVelocity(M_PI/1.5);
	posori_task->_otg->setMaxAngularAcceleration(M_PI/3);
	posori_task->_otg->setMaxAngularJerk(M_PI);

	// force sensor setup
	Affine3d sensor_frame = Affine3d::Identity();
	sensor_frame.translation() = Vector3d(0, 0, 0.12);
	posori_task->setForceSensorFrame(link_name, sensor_frame);
	VectorXd sensed_force_moment = VectorXd::Zero(6);

	VectorXd force_sensor_bias = VectorXd::Zero(6);
	double ee_mass = 0;
	Vector3d ee_com_in_sensor_frame = Vector3d::Zero();

	if(!flag_simulation)
	{
		force_sensor_bias << -0.911605,    1.6281,  -0.29196, -0.104877,  0.416328, 0.0185848;
		ee_mass =  0.249114;
		ee_com_in_sensor_frame = Vector3d(-0.000571861,  -0.00177381,    0.0725383);
	}

	double force_detection_treshold = 8.0;

	// setup trajectories for cleaning and drying window
	bool alignment_1_needed = true;
	bool alignment_2_needed = true;
	bool wait_for_alignment = false;
	int wait_for_alignment_counter = 1000;

	Vector3d initial_pos_clean_dry = Vector3d::Zero();

	vector<Vector3d> clean_window_increments_in_world;
	clean_window_increments_in_world.push_back(Vector3d(0.0, 0.40, 0.0));
	clean_window_increments_in_world.push_back(Vector3d(0.0, -0.40, -0.07));
	clean_window_increments_in_world.push_back(Vector3d(0.0, 0.40, -0.07));
	clean_window_increments_in_world.push_back(Vector3d(0.0, -0.40, 0.0));

	int n_clean_via_points = clean_window_increments_in_world.size();
	int current_clean_via_point = 0;

	bool switch_tool = false;

	vector<Vector3d> dry_window_increments_in_world;
	dry_window_increments_in_world.push_back(Vector3d( 0.00, 0.40, 0.0));
	dry_window_increments_in_world.push_back(Vector3d( 0.05, 0.0, 0.0));
	dry_window_increments_in_world.push_back(Vector3d( 0.00,-0.40,-0.08));
	dry_window_increments_in_world.push_back(Vector3d(-0.09, 0.0, 0.0));
	dry_window_increments_in_world.push_back(Vector3d( 0.00, 0.40, 0.0));
	dry_window_increments_in_world.push_back(Vector3d( 0.05, 0.0, 0.0));
	dry_window_increments_in_world.push_back(Vector3d( 0.00,-0.40,-0.08));
	dry_window_increments_in_world.push_back(Vector3d(-0.09, 0.0, 0.0));
	dry_window_increments_in_world.push_back(Vector3d( 0.00, 0.40, 0.0));
	dry_window_increments_in_world.push_back(Vector3d( 0.05, 0.0, 0.0));

	int n_dry_via_points = dry_window_increments_in_world.size();
	int current_dry_via_point = -2;

	// setup redis exchanges
	redis_client.addEigenToRead(JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToRead(JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToRead(FORCE_SENSED_KEY, sensed_force_moment);
	if(!flag_simulation)
	{
		redis_client.addEigenToRead(MASSMATRIX_KEY, robot->_M);
		redis_client.addEigenToRead(CORIOLIS_KEY, coriolis);

	}

	redis_client.addEigenToWrite(JOINT_TORQUES_COMMANDED_KEY, command_torques);

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
		redis_client.readAllSetupValues();

		// update robot model
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);
		}
		else
		{
			robot->updateKinematics();
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			// cout << robot->_M << endl;
			// robot->_M = Eigen::MatrixXd::Identity(dof, dof);
			robot->_M_inv = robot->_M.inverse();
		}

		// read force sensor data and remove bias and effects from hand gravity
		sensed_force_moment -= force_sensor_bias; 
		Matrix3d R_sensor = Matrix3d::Identity();
		robot->rotation(R_sensor, link_name);
		Vector3d p_tool_sensorFrame = ee_mass * R_sensor.transpose() * Vector3d(0,0,-9.81); 
		sensed_force_moment.head(3) += p_tool_sensorFrame;
		sensed_force_moment.tail(3) += ee_com_in_sensor_frame.cross(p_tool_sensorFrame);

		posori_task->updateSensedForceAndMoment(sensed_force_moment.head(3), sensed_force_moment.tail(3));


		if(state == MOVE_TO_INITIAL)
		{
			// update tasks model
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			if((joint_task->_desired_position - joint_task->_current_position).norm() < 1e-1)
			{
				joint_task->_desired_position = target_q;
				joint_task->_ki = 0.0;

				posori_task->reInitializeTask();
				posori_task->_desired_position += R_robot_world * Vector3d(-0.1, 0.0, 0.0);
				state = ALIGNMENT_PHASE;
			}
		}

		else if(state == ALIGNMENT_PHASE)
		{
			// update tasks model
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;

			// mode switch
			Vector3d sensed_force_in_world_frame = robot_pose_in_world.linear() * posori_task->_sensed_force;
			if(sensed_force_in_world_frame(0) < -force_detection_treshold && alignment_1_needed)
			{
				Vector3d force_axis_in_robot_frame = R_robot_world * Vector3d::UnitX();
				posori_task->setForceAxis(force_axis_in_robot_frame);
				posori_task->_desired_force = R_robot_world * Vector3d(-force_detection_treshold, 0.0, 0.0);

				Vector3d moment_axis_in_robot_frame = R_robot_world * Vector3d::UnitY();
				posori_task->setMomentAxis(moment_axis_in_robot_frame);
				posori_task->setClosedLoopMomentControl();

				// alignment_1_needed = false;
				wait_for_alignment = true;
			}

			if(!alignment_1_needed && alignment_2_needed)
			{
				posori_task->_desired_position += R_robot_world * Vector3d(0.0, -0.00005, 0.0);
			}

			if(alignment_2_needed && sensed_force_in_world_frame(1) < -force_detection_treshold)
			{
				Vector3d motion_axis_in_robot_frame = R_robot_world * Vector3d::UnitZ();
				posori_task->setLinearMotionAxis(motion_axis_in_robot_frame);
				posori_task->_desired_force = R_robot_world * Vector3d(-force_detection_treshold, -force_detection_treshold, 0.0);

				Vector3d rotmot_axis_in_robot_frame = R_robot_world * Vector3d::UnitZ();
				posori_task->setAngularMotionAxis(rotmot_axis_in_robot_frame);	

				alignment_2_needed = false;
				wait_for_alignment = true;
			}

			if(wait_for_alignment)
			{
				wait_for_alignment_counter--;
			}

			if(wait_for_alignment_counter < 0 && alignment_1_needed)
			{
				alignment_1_needed = false;
				wait_for_alignment = false;
				wait_for_alignment_counter = 1000;
			}

			if(wait_for_alignment_counter < 0 && !alignment_1_needed)
			{
				posori_task->reInitializeTask();
				initial_pos_clean_dry = posori_task->_current_position;

				Vector3d force_axis_in_robot_frame = R_robot_world * Vector3d::UnitX();
				posori_task->setForceAxis(force_axis_in_robot_frame);
				posori_task->_desired_force = R_robot_world * Vector3d(-force_detection_treshold, 0.0, 0.0);

				Vector3d moment_axis_in_robot_frame = R_robot_world * Vector3d::UnitY();
				posori_task->setMomentAxis(moment_axis_in_robot_frame);
				posori_task->setClosedLoopMomentControl();

				state = CLEAN_WINDOW;
			}



			// // cout << sensed_force_moment.transpose() << endl;
			// cout << posori_task->_sensed_force.transpose() << endl;
			// cout << sensed_force_in_world_frame.transpose() << endl;
			// cout << endl;

		}

		else if(state == CLEAN_WINDOW)
		{
			// update tasks model
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;

			// via points
			if(posori_task->goalPositionReached(0.015))
			{
				if(current_clean_via_point < n_clean_via_points)
				{
					posori_task->_desired_position += R_robot_world * clean_window_increments_in_world[current_clean_via_point];
				}
				else if(current_clean_via_point == n_clean_via_points)
				{
					posori_task->setFullLinearMotionControl();
					posori_task->_desired_position = initial_pos_clean_dry + R_robot_world * Vector3d(0.15, 0.0, 0.0);
				}
				else
				{
					joint_task->reInitializeTask();
					joint_task->_desired_position(dof-1) += M_PI;
					joint_task->_ki = 25.0;

					state = SWITCH_TOOL;
				}
				
				current_clean_via_point++;
			}

		}

		else if(state == SWITCH_TOOL)
		{
			// update tasks model
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			if((joint_task->_current_position - joint_task->_desired_position).norm() < 1e-1)
			{
				joint_task->_desired_position = target_q;
				joint_task->_ki = 0.0;

				posori_task->reInitializeTask();
				posori_task->_desired_position = initial_pos_clean_dry + R_robot_world*Vector3d(-0.07, 0.05, 0.0);

				state = DRY_WINDOW;
			}			
		}

		else if(state == DRY_WINDOW)
		{
			// update tasks model
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;
			
			Vector3d sensed_force_in_world_frame = robot_pose_in_world.linear() * posori_task->_sensed_force;
			if(current_dry_via_point == -2 && sensed_force_in_world_frame(0) < -force_detection_treshold)
			{
				posori_task->_desired_position = initial_pos_clean_dry; // + R_robot_world * Vector3d(0.0, -0.00002, 0.0);
				Vector3d force_axis_in_robot_frame = R_robot_world * Vector3d::UnitX();
				posori_task->setForceAxis(force_axis_in_robot_frame);
				posori_task->_desired_force = R_robot_world * Vector3d(-force_detection_treshold, 0.0, 0.0);

				Vector3d moment_axis_in_robot_frame = R_robot_world * Vector3d::UnitY();
				posori_task->setMomentAxis(moment_axis_in_robot_frame);
				posori_task->setClosedLoopMomentControl();

				wait_for_alignment = false;
				current_dry_via_point = -1;

			}
			else if(current_dry_via_point == -1 && sensed_force_in_world_frame(1) >= -force_detection_treshold && !wait_for_alignment)
			{
				posori_task->_desired_position += R_robot_world * Vector3d(0.0, -0.000002, 0.0);
			}
			else if(current_dry_via_point == -1 && sensed_force_in_world_frame(1) < -force_detection_treshold && !wait_for_alignment)
			{
				Vector3d motion_axis_in_robot_frame = R_robot_world * Vector3d::UnitZ();
				posori_task->setLinearMotionAxis(motion_axis_in_robot_frame);
				posori_task->_desired_force = R_robot_world * Vector3d(-force_detection_treshold, -force_detection_treshold, 0.0);

				Vector3d rotmot_axis_in_robot_frame = R_robot_world * Vector3d::UnitZ();
				posori_task->setAngularMotionAxis(rotmot_axis_in_robot_frame);					

				wait_for_alignment = true;
				wait_for_alignment_counter = 1000;
			}
			else if(current_dry_via_point == -1 && wait_for_alignment)
			{
				if(wait_for_alignment_counter > 0)
				{
					wait_for_alignment_counter--;
				}
				else
				{
					posori_task->reInitializeTask();
					initial_pos_clean_dry = posori_task->_current_position;

					Vector3d force_axis_in_robot_frame = R_robot_world * Vector3d::UnitX();
					posori_task->setForceAxis(force_axis_in_robot_frame);
					posori_task->_desired_force = R_robot_world * Vector3d(-force_detection_treshold, 0.0, 0.0);

					Vector3d moment_axis_in_robot_frame = R_robot_world * Vector3d::UnitY();
					posori_task->setMomentAxis(moment_axis_in_robot_frame);
					posori_task->setClosedLoopMomentControl();

					current_dry_via_point = 0;
				}
			}
			else if((current_dry_via_point % 4) == 0 && sensed_force_in_world_frame(0) < -force_detection_treshold)
			{
				Vector3d force_axis_in_robot_frame = R_robot_world * Vector3d::UnitX();
				posori_task->setForceAxis(force_axis_in_robot_frame);
				posori_task->_desired_force = R_robot_world * Vector3d(-force_detection_treshold, 0.0, 0.0);
				posori_task->_desired_position += R_robot_world * dry_window_increments_in_world[current_dry_via_point];

				current_dry_via_point++;
			}

			// via points
			if(current_dry_via_point > 0 && posori_task->goalPositionReached(0.015))
			{
				// if(current_dry_via_point < n_dry_via_points && (current_dry_via_point % 4) == 0)
				// {

				// }
				if(current_dry_via_point < n_dry_via_points && (current_dry_via_point % 4) > 0)
				{
					posori_task->setFullLinearMotionControl();
					posori_task->_desired_position += R_robot_world * dry_window_increments_in_world[current_dry_via_point];
				}
				else if(current_dry_via_point == n_dry_via_points)
				{
					posori_task->setFullLinearMotionControl();
					posori_task->_desired_position = initial_pos_clean_dry + R_robot_world * Vector3d(0.15, 0.15, 0.0);

					state = END1;
				}
				
				current_dry_via_point++;
			}
		}

		else if(state == END1)
		{
			// update tasks model
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;	

			if(posori_task->goalPositionReached(0.01))
			{
				joint_task->reInitializeTask();
				joint_task->_desired_position(dof-1) -= M_PI;
				state = END2;
			}		
		}

		else if(state == END2)
		{
			// update tasks model
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;				
		}

		// send to redis
		redis_client.writeAllSetupValues();

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
