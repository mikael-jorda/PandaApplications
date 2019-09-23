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

#define IMPEDANCE                           0
#define DYNAMIC_DECOUPLING                  1
#define INERTIA_SATURATION                  2
#define INERTIA_REGULARIZATION              3
#define INERTIA_REGULARIZATION_ORI_ONLY     4
#define IMPEDANCE_ORI_POSORI_DECOUPLING     5
#define IMPEDANCE_ORI_NO_POSORI_DECOUPLING  6

#define SPIRAL_Z                 0
#define SPIRAL_X                 1
#define ORIENTATION_ONLY         2
#define SPIRAL_Z_ORIENTATION     3
#define SPIRAL_X_ORIENTATION     4

#define SLOW_SPEED    0
#define MEDIUM_SPEED  1
#define HIGH_SPEED    2

#define LOW_GAINS        0
#define MEDIUM_GAINS     1
#define HIGH_GAINS       2

#define DEBUG                           0
#define EXPERIMENT_TRACKING             1
#define EXPERIMENT_REGULATION           2
#define GO_TO_INITIAL                   3

int state = GO_TO_INITIAL;
int controller_type = 0;
int speed = 0;
int gains_value = 0;
int motion_type = 0;

int experiment_number = 1;
int number_of_oscillation_per_experiment = 2;

// linear motion parameters
Vector3d experiment_speed_circle_velocity = Vector3d(0.1, 0.3, 0.5);
Vector3d experiment_speed_axis_max_velocity = Vector3d(0.05, 0.25, 0.45);
double circle_radius = 0.06;
double cylinder_length = 0.20;
Vector3d experiment_speed_frequency_circle = experiment_speed_circle_velocity / circle_radius;
Vector3d experiment_speed_frequency_axis_motion = experiment_speed_axis_max_velocity / cylinder_length;

// orientation part parameters
Vector3d experiment_speed_orientation_oscillation_frequency = 2 * M_PI * Vector3d(0.1, 0.3, 0.5);
Vector3d orientation_rotation_axis = Vector3d::UnitX();
double orientation_oscillation_amplitude = M_PI/3;

int experiment_countdown = 0;

// gains
Vector3d kp_experiments = Vector3d(100.0, 200.0, 400.0);
Vector3d kv_experiments = Vector3d(20.0, 25.0, 30.0);

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

const string DESIRED_POSITION_KEY = "sai2::PandaApplication::controller::desired_ee_pos";
const string CURRENT_POSITION_KEY = "sai2::PandaApplication::controller::current_ee_pos";

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const string prefix_path = "../../00-experiment_pose_control/data_files/";
string create_filename();

void write_first_line(ofstream& file_handler);

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

	robot->updateModel();

	// prepare controller	
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	VectorXd initial_q = VectorXd::Zero(dof);
	initial_q << 20.0, 25.0, 5.0, -110.0, 30.0, 125.0, 45.0;
	initial_q *= M_PI/180.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = true;
	joint_task->_kp = 200.0;
	joint_task->_kv = 25.0;
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
	bool newfile = true;
	string filename = create_filename();
	ofstream data_file;

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
				if(motion_type < 5)
				{
					posori_task->reInitializeTask();
					x_init = posori_task->_current_position;
					R_init = posori_task->_current_orientation;

					joint_task->_ki = 0;
					joint_task->_use_interpolation_flag = false;

					state = EXPERIMENT_TRACKING;
					initial_time = current_time;
				}
			}
		}

		else
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// if(controller_counter % 100 == 0)
			// {
			// 	cout << "Lambda : \n" << posori_task->_Lambda << endl << endl;
			// }

			// set gains
			posori_task->_kp_pos = kp_experiments(gains_value);
			posori_task->_kv_pos = kv_experiments(gains_value);
			posori_task->_kp_ori = kp_experiments(gains_value);
			posori_task->_kv_ori = kv_experiments(gains_value);
		
			redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
			redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));

			// set experiment type
			if(state == DEBUG)
			{
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);

				joint_task->_desired_position = initial_q;

				posori_task->computeTorques(posori_task_torques);

				cout << posori_task->_current_position.transpose() << '\n' << posori_task->_current_orientation << endl << endl;

			}
			else if (state == EXPERIMENT_REGULATION)
			{
				// read force sensor
			}
			else if(state == EXPERIMENT_TRACKING)
			{

				if(newfile)
				{
					data_file.open(create_filename());
					write_first_line(data_file);
					newfile = false;
				}

				data_file << posori_task->_desired_position.transpose() << '\t' << posori_task->_current_position.transpose() << '\t' << 
					posori_task->_desired_velocity.transpose() << '\t' << posori_task->_current_velocity.transpose() << '\t' <<
					posori_task->_orientation_error.transpose() << '\t' << posori_task->_desired_angular_velocity.transpose() << '\t' <<
					posori_task->_current_angular_velocity.transpose() << '\t' << posori_task_torques.transpose() << command_torques.transpose() <<
					endl;

				if(motion_type == SPIRAL_Z)
				{
					posori_task->_desired_position(0) = x_init(0) -
						circle_radius * (1-cos(experiment_speed_frequency_circle(speed)*(current_time - initial_time)));
					posori_task->_desired_position(1) = x_init(1) +
						circle_radius * sin(experiment_speed_frequency_circle(speed)*(current_time - initial_time));
					posori_task->_desired_position(2) = x_init(2) +
						cylinder_length/2 *(1-cos(experiment_speed_frequency_axis_motion(speed)*(current_time - initial_time)));
				}
				else if(motion_type == SPIRAL_X)
				{
					posori_task->_desired_position(0) = x_init(0) -
						cylinder_length/2 *(1-cos(experiment_speed_frequency_axis_motion(speed)*(current_time - initial_time)));
					posori_task->_desired_position(1) = x_init(1) -
						circle_radius * sin(experiment_speed_frequency_circle(speed)*(current_time - initial_time));
					posori_task->_desired_position(2) = x_init(2) +
						circle_radius * (1-cos(experiment_speed_frequency_circle(speed)*(current_time - initial_time)));
				}
				else if(motion_type == ORIENTATION_ONLY)
				{
					double angle = orientation_oscillation_amplitude * sin(experiment_speed_orientation_oscillation_frequency(speed)*(current_time - initial_time));
					Matrix3d R_curr = AngleAxisd(angle, orientation_rotation_axis).toRotationMatrix();

					posori_task->_desired_orientation = R_curr * R_init;
				}
				else if(motion_type == SPIRAL_Z_ORIENTATION)
				{
					posori_task->_desired_position(0) = x_init(0) -
						circle_radius * (1-cos(experiment_speed_frequency_circle(speed)*(current_time - initial_time)));
					posori_task->_desired_position(1) = x_init(1) +
						circle_radius * sin(experiment_speed_frequency_circle(speed)*(current_time - initial_time));
					posori_task->_desired_position(2) = x_init(2) +
						cylinder_length/2 *(1-cos(experiment_speed_frequency_axis_motion(speed)*(current_time - initial_time)));
				
					double angle = orientation_oscillation_amplitude * sin(experiment_speed_orientation_oscillation_frequency(speed)*(current_time - initial_time));
					Matrix3d R_curr = AngleAxisd(angle, orientation_rotation_axis).toRotationMatrix();

					posori_task->_desired_orientation = R_curr * R_init;
				}
				else if(motion_type == SPIRAL_X_ORIENTATION)
				{
					posori_task->_desired_position(0) = x_init(0) -
						cylinder_length/2 *(1-cos(experiment_speed_frequency_axis_motion(speed)*(current_time - initial_time)));
					posori_task->_desired_position(1) = x_init(1) -
						circle_radius * sin(experiment_speed_frequency_circle(speed)*(current_time - initial_time));
					posori_task->_desired_position(2) = x_init(2) +
						circle_radius * (1-cos(experiment_speed_frequency_circle(speed)*(current_time - initial_time)));
				
					double angle = orientation_oscillation_amplitude * sin(experiment_speed_orientation_oscillation_frequency(speed)*(current_time - initial_time));
					Matrix3d R_curr = AngleAxisd(angle, orientation_rotation_axis).toRotationMatrix();

					posori_task->_desired_orientation = R_curr * R_init;
				}

				// switch scenario
				if(experiment_countdown == 0)
				{

					data_file.close();

					cout << "finished experiment : " << experiment_number << "/315" << endl;
					cout << "motion type : " << motion_type << "\tspeed : " << speed << "\tcontroller type : " << controller_type << "\tgains : " << gains_value << endl;
					cout << endl;
					newfile = true;
					experiment_number++;
					gains_value++;
					if(gains_value == 3)
					{
						gains_value = 0;
						controller_type++;
						if(controller_type == 7)
						// if(controller_type == 1)
						{
							controller_type = 0;
							speed++;

							state = GO_TO_INITIAL;

							joint_task->_use_interpolation_flag = true;

							if(speed == 3)
							{
								speed = 0;
								motion_type++;
							}

						}
					}
					if(motion_type == ORIENTATION_ONLY)
					{
						experiment_countdown = 1000 * number_of_oscillation_per_experiment * 2 * M_PI / experiment_speed_orientation_oscillation_frequency(speed);
					}
					else
					{
						experiment_countdown = 1000 * number_of_oscillation_per_experiment * 2 * M_PI / experiment_speed_frequency_axis_motion(speed);
					}

				}
				experiment_countdown--;
			}
		
			// set controller type
			if(controller_type == IMPEDANCE)
			{
				posori_task->_kp_pos = kp_experiments(gains_value) * 7;
				posori_task->_kv_pos = kv_experiments(gains_value) * 7;
				posori_task->_kp_ori = kp_experiments(gains_value) * 0.3;
				posori_task->_kv_ori = kv_experiments(gains_value) * 0.3;

				posori_task->_Lambda.setIdentity();
				posori_task->computeTorques(posori_task_torques);
			}
			else if(controller_type == DYNAMIC_DECOUPLING)
			{
				posori_task->computeTorques(posori_task_torques);
			}
			else if(controller_type == INERTIA_SATURATION)
			{
				for(int i=0 ; i<6 ; i++)
				{
					if(posori_task->_Lambda(i,i) < 0.1)
					{
						posori_task->_Lambda(i,i) = 0.1;
					}
				}
				posori_task->computeTorques(posori_task_torques);
			}
			else if(controller_type == INERTIA_REGULARIZATION)
			{
				for(int i=0 ; i<6 ; i++)
				{
					posori_task->_Lambda(i,i) += 0.1;
				}
				posori_task->computeTorques(posori_task_torques);
			}
			else if(controller_type == INERTIA_REGULARIZATION_ORI_ONLY)
			{
				for(int i=3 ; i<6 ; i++)
				{
					posori_task->_Lambda(i,i) += 0.1;
				}
				posori_task->computeTorques(posori_task_torques);
			}
			else if(controller_type == IMPEDANCE_ORI_POSORI_DECOUPLING)
			{
				posori_task->_kp_ori = kp_experiments(gains_value) * 0.3;
				posori_task->_kv_ori = kv_experiments(gains_value) * 0.3;

				posori_task->_Lambda.block<3,3>(3,3) = Matrix3d::Identity();
				posori_task->computeTorques(posori_task_torques);
			}
			else if(controller_type == IMPEDANCE_ORI_NO_POSORI_DECOUPLING)
			{
				posori_task->_kp_ori = kp_experiments(gains_value) * 0.3;
				posori_task->_kv_ori = kv_experiments(gains_value) * 0.3;

				posori_task->_Lambda.block<3,3>(3,3) = Matrix3d::Identity();
				posori_task->_Lambda.block<3,3>(0,3) = Matrix3d::Zero();
				posori_task->_Lambda.block<3,3>(3,0) = Matrix3d::Zero();
				posori_task->computeTorques(posori_task_torques);
			}
			else
			{
				command_torques.setZero(dof);
			}

			// send to redis
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques + coriolis;
		}

		// command_torques.setZero(dof);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		redis_client.setEigenMatrixJSON(DESIRED_POSITION_KEY, posori_task->_desired_position);
		redis_client.setEigenMatrixJSON(CURRENT_POSITION_KEY, posori_task->_current_position);

		// write file

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

string create_filename()
{
	string return_filename = prefix_path;

	switch(state)
	{
		case EXPERIMENT_REGULATION :
			return_filename += "regulation_";
			break;
		case EXPERIMENT_TRACKING :
			return_filename += "tracking_";
			break;
		case DEBUG :
			return_filename += "debug_";
			break;
		default :
			return_filename += "should_not_happen_";
			break;
	}

	switch(motion_type)
	{
		case SPIRAL_Z :
			return_filename += "spiralZ_";
			break;
		case SPIRAL_X :
			return_filename += "spiralX_";
			break;
		case ORIENTATION_ONLY :
			return_filename += "orientationOnly_";
			break;		
		case SPIRAL_Z_ORIENTATION :
			return_filename += "spiralZOrientation_";
			break;
		case SPIRAL_X_ORIENTATION :
			return_filename += "spiralXOrientation_";
			break;
	}

	switch(speed)
	{
		case SLOW_SPEED :
			return_filename += "slow_";
			break;
		case MEDIUM_SPEED :
			return_filename += "medium_";
			break;
		case HIGH_SPEED :
			return_filename += "fast_";
			break;
	}

	switch(controller_type)
	{
		case IMPEDANCE :
			return_filename += "impedance_";
			break;
		case DYNAMIC_DECOUPLING :
			return_filename += "dynamicDecoupling_";
			break;
		case INERTIA_SATURATION :
			return_filename += "inertiaSaturation_";
			break;
		case INERTIA_REGULARIZATION :
			return_filename += "inertiaRegularization_";
			break;
		case INERTIA_REGULARIZATION_ORI_ONLY :
			return_filename += "inertiaRegularizationOriOnly_";
			break;
		case IMPEDANCE_ORI_POSORI_DECOUPLING :
			return_filename += "impedanceOriDecouplingPosori_";
			break;
		case IMPEDANCE_ORI_NO_POSORI_DECOUPLING :
			return_filename += "impedanceOriNoDecouplingPosori_";
			break;
	}

	switch(gains_value)
	{
		case LOW_GAINS :
			return_filename += "lowGains";
			break;
		case MEDIUM_GAINS :
			return_filename += "mediumGains";
			break;
		case HIGH_SPEED :
			return_filename += "highGains";
			break;
	}

	return_filename += ".txt";

	return return_filename;

}

void write_first_line(ofstream& file_handler)
{
	switch(state)
	{
		case EXPERIMENT_REGULATION :
			file_handler << "desired_position[3]\tcurrent_position[3]\tdesired_velocity[3]\tcurrent_velocity[3]\t" <<
				"orientation_error[3]\tdesired_angular_velocity[3]\tcurrent_angular_velocity[3]\t" <<
				"posori_task_torques[7]\tcommand_torques[7]\tsensed_force[6]\n";
			break;
		case EXPERIMENT_TRACKING :
			file_handler << "desired_position[3]\tcurrent_position[3]\tdesired_velocity[3]\tcurrent_velocity[3]\t" <<
				"orientation_error[3]\tdesired_angular_velocity[3]\tcurrent_angular_velocity[3]\t" <<
				"posori_task_torques[7]\tcommand_torques[7]\n";
			break;
		case DEBUG :
			file_handler << "debug_file\n";
			break;
	}

}
