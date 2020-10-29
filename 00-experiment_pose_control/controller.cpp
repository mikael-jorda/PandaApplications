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

const vector<string> vec_controller_types = {
	"inertiaSaturation",
	"noDynDecoupling",
	"partialDynDecoupling",
	"fullDynDecoupling",
};

const vector<string> vec_trajectory_types = {
	// "spiralZ",
	// "spiralX",
	// "orientationOnly",
	// "spiralZOrientation",
	"spiralXOrientation",
};

const vector<string> vec_speeds = {
	"slowSpeed",
	"mediumSpeed",
	"fastSpeed",
};

const vector<string> vec_gain_values = {
	"lowGains",
	"mediumGains",
	"highGains",
};



#define DEBUG                           0
#define GO_TO_INITIAL                   1
#define EXPERIMENT                      2

const int number_trajectory_types = vec_trajectory_types.size();
const int number_speeds = vec_speeds.size();
const int number_controller_types = vec_controller_types.size();
const int number_gains_values = vec_gain_values.size();


int state = GO_TO_INITIAL;
int trajectory_type = 0;
int speed = 2;
int controller_type = 0;
int gains_value = 0;

int experiment_number = 1 + gains_value + number_gains_values * (controller_type + number_controller_types * (speed + number_speeds * trajectory_type));
const int number_experiments = number_gains_values * number_controller_types * number_speeds * number_trajectory_types;
const int number_of_oscillation_per_experiment = 2;


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

// gains
const string KP_JOINT_KEY = "sai2::PandaApplication::controller:kp_joint";
const string KV_JOINT_KEY = "sai2::PandaApplication::controller:kv_joint";

const string DESIRED_POSITION_KEY = "sai2::PandaApplication::controller::desired_ee_pos";
const string CURRENT_POSITION_KEY = "sai2::PandaApplication::controller::current_ee_pos";

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const string prefix_path = "../../00-experiment_pose_control/data_files/data/exp_tracking_";
string create_filename();

void write_first_line(ofstream& file_handler);

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


	// setup experiment parameters
	// Vector3d experiment_speed_circle_velocity = Vector3d(0.1, 0.25, 0.4);
	// Vector3d experiment_speed_axis_max_velocity = Vector3d(0.1, 0.2, 0.3);
	VectorXd experiment_speed_circle_velocity = VectorXd::Zero(number_speeds);
	VectorXd experiment_speed_axis_max_velocity = VectorXd::Zero(number_speeds);
	VectorXd experiment_speed_orientation_oscillation_frequency = VectorXd::Zero(number_speeds);

	for(int i=0 ; i<number_speeds ; i++)
	{
		if(vec_speeds[i] == "slowSpeed")
		{
			experiment_speed_circle_velocity(i) = 0.1;
			experiment_speed_axis_max_velocity(i) = 0.1;
			experiment_speed_orientation_oscillation_frequency(i) = 2 * M_PI * 0.05;
		}
		else if(vec_speeds[i] == "mediumSpeed")
		{
			experiment_speed_circle_velocity(i) = 0.25;
			experiment_speed_axis_max_velocity(i) = 0.2;
			experiment_speed_orientation_oscillation_frequency(i) = 2 * M_PI * 0.15;
		}
		else if(vec_speeds[i] == "fastSpeed")
		{
			experiment_speed_circle_velocity(i) = 0.3;
			experiment_speed_axis_max_velocity(i) = 0.4;
			experiment_speed_orientation_oscillation_frequency(i) = 2 * M_PI * 0.25;
		}
		else
		{
			throw "invalid speed";
		}
	}

	double circle_radius = 0.06;
	double cylinder_length = 0.20;
	VectorXd experiment_speed_frequency_circle = experiment_speed_circle_velocity / circle_radius;
	VectorXd experiment_speed_frequency_axis_motion = experiment_speed_axis_max_velocity / cylinder_length;

	// orientation part parameters
	// Vector3d experiment_speed_orientation_oscillation_frequency = 2 * M_PI * Vector3d(0.05, 0.15, 0.25);
	Vector3d orientation_rotation_axis = Vector3d::UnitX();
	double orientation_oscillation_amplitude = M_PI/4;

	int experiment_countdown = 1000 * number_of_oscillation_per_experiment * 2 * M_PI / experiment_speed_frequency_axis_motion(speed);

	// gains
	// Vector3d kp_experiments = Vector3d(100.0, 200.0, 300.0);
	// Vector3d kv_experiments = Vector3d(15.0, 20.0, 25.0);

	VectorXd kp_experiments = VectorXd::Zero(number_gains_values);
	VectorXd kv_experiments = VectorXd::Zero(number_gains_values);

	for(int i=0 ; i<number_gains_values ; i++)
	{
		if(vec_gain_values[i] == "lowGains")
		{
			kp_experiments(i) = 100.0;
			kv_experiments(i) = 15.0;
		}
		else if(vec_gain_values[i] == "mediumGains")
		{
			kp_experiments(i) = 200.0;
			kv_experiments(i) = 20.0;
		}
		else if(vec_gain_values[i] == "highGains")
		{
			kp_experiments(i) = 300.0;
			kv_experiments(i) = 25.0;
		}
		else
		{
			throw "invalid gains";
		}
	}


	// cout << "speed circle : " << experiment_speed_circle_velocity.transpose() << endl;
	// cout << "speed axis : " << experiment_speed_axis_max_velocity.transpose() << endl;
	// cout << "speed orientation : " << experiment_speed_orientation_oscillation_frequency.transpose() << endl;
	// cout << "kp : " << kp_experiments.transpose() << endl;
	// cout << "kv : " << kv_experiments.transpose() << endl;
	// cout << endl;

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
	// initial_q << 20.0, 25.0, 5.0, -110.0, 30.0, 125.0, 45.0;
	initial_q << 20.0, 45.0, 5.0, -90.0, 30.0, 125.0, 45.0;
	initial_q *= M_PI/180.0;

	// initial_q << -0.453204,0.739596,0.94597,-1.55224,0.0779967,1.75418,0.685897;

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
				if(trajectory_type < number_trajectory_types)
				{
					posori_task->reInitializeTask();
					x_init = posori_task->_current_position;
					R_init = posori_task->_current_orientation;

					joint_task->_ki = 0;
					joint_task->_use_interpolation_flag = false;

					state = EXPERIMENT;
					initial_time = current_time;
				}
				else if(trajectory_type == number_trajectory_types)
				{
					cout << "\n\nALL EXPERIMENTS FINISHED\n\n" << endl;
					trajectory_type = number_trajectory_types + 1;
				}
			}
		}

		else
		{

			// set gains
			posori_task->_kp_pos = kp_experiments(gains_value);
			posori_task->_kv_pos = kv_experiments(gains_value);
			posori_task->_kp_ori = kp_experiments(gains_value);
			posori_task->_kv_ori = kv_experiments(gains_value);
		
			redis_client.set(KP_JOINT_KEY, to_string(joint_task->_kp));
			redis_client.set(KV_JOINT_KEY, to_string(joint_task->_kv));


			// set controller type
			if(vec_controller_types[controller_type] == "noDynDecoupling")
			{
				posori_task->_kp_pos = kp_experiments(gains_value) * 5;
				posori_task->_kv_pos = kv_experiments(gains_value) * 5;
				posori_task->_kp_ori = kp_experiments(gains_value) * 0.2;
				posori_task->_kv_ori = kv_experiments(gains_value) * 0.2;

				// posori_task->_kp_pos = kp_experiments(gains_value) * 7;
				// posori_task->_kv_pos = kv_experiments(gains_value) * 9;
				// posori_task->_kp_ori = kp_experiments(gains_value) * 0.10;
				// posori_task->_kv_ori = kv_experiments(gains_value) * 0.18;

				posori_task->setDynamicDecouplingNone();
			}
			else if(vec_controller_types[controller_type] == "fullDynDecoupling")
			{
				posori_task->setDynamicDecouplingFull();
			}
			else if(vec_controller_types[controller_type] == "partialDynDecoupling")
			{
				posori_task->_kp_ori = kp_experiments(gains_value) * 0.20;
				posori_task->_kv_ori = kv_experiments(gains_value) * 0.20;

				posori_task->setDynamicDecouplingPartial();
			}
			else if(vec_controller_types[controller_type] == "inertiaSaturation")
			{
				posori_task->setDynamicDecouplingInertiaSaturation();
			}
			else
			{
				throw "dynamic decoupling incompatible";
			}


			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// if(controller_counter % 100 == 0)
			// {
			// 	cout << "Lambda : \n" << posori_task->_Lambda << endl << endl;
			// }

			// set experiment type
			if(state == DEBUG)
			{
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);

				joint_task->_desired_position = initial_q;

				posori_task->computeTorques(posori_task_torques);

				cout << posori_task->_current_position.transpose() << '\n' << posori_task->_current_orientation << endl << endl;

			}
			else if(state == EXPERIMENT)
			{

				if(newfile)
				{
					data_file.open(create_filename());
					write_first_line(data_file);
					newfile = false;
				}

				// cout << "mass matrix :\n" << robot->_M << endl;
				// cout << "mass matrix :\n" << posori_task->_jacobian << endl;


				data_file << robot->_q.transpose() << '\t' << robot->_dq.transpose() << '\t' << posori_task->_desired_position.transpose() << '\t' << posori_task->_current_position.transpose() << '\t' << 
					posori_task->_desired_velocity.transpose() << '\t' << posori_task->_current_velocity.transpose() << '\t' <<
					posori_task->_orientation_error.transpose() << '\t' << posori_task->_desired_angular_velocity.transpose() << '\t' <<
					posori_task->_current_angular_velocity.transpose() << '\t' << posori_task->_task_force.transpose() << '\t' << posori_task_torques.transpose() << '\t' << command_torques.transpose() << '\t';

					for(int k=0 ; k<dof ; k++)
					{
						data_file << robot->_M.col(k).transpose() << '\t';
					}
					for(int k=0 ; k<dof ; k++)
					{
						data_file << posori_task->_jacobian.col(k).transpose() << '\t';
					}
					data_file << endl;


				if(vec_trajectory_types[trajectory_type] == "spiralZ")
				{
					double w_circle = experiment_speed_frequency_circle(speed);
					double w_axis = experiment_speed_frequency_axis_motion(speed);
					double time = current_time - initial_time;

					posori_task->_desired_position(0) = x_init(0) -	circle_radius * (1-cos(w_circle * time));
					posori_task->_desired_position(1) = x_init(1) +	circle_radius * sin(w_circle * time);
					posori_task->_desired_position(2) = x_init(2) +	cylinder_length/2 *(1-cos(w_axis * time));

					posori_task->_desired_velocity(0) = - circle_radius * w_circle * sin(w_circle * time);
					posori_task->_desired_velocity(1) =   circle_radius * w_circle * cos(w_circle * time);
					posori_task->_desired_velocity(2) =   cylinder_length/2 * w_axis * sin(w_axis * time);

					posori_task->_desired_acceleration(0) = - circle_radius * w_circle * w_circle * cos(w_circle * time);
					posori_task->_desired_acceleration(1) = - circle_radius * w_circle * w_circle * sin(w_circle * time);
					posori_task->_desired_acceleration(2) =   cylinder_length/2 * w_axis * w_axis * cos(w_axis * time);

				}
				else if(vec_trajectory_types[trajectory_type] == "spiralX")
				{
					double w_circle = experiment_speed_frequency_circle(speed);
					double w_axis = experiment_speed_frequency_axis_motion(speed);
					double time = current_time - initial_time;

					posori_task->_desired_position(0) = x_init(0) - cylinder_length/2 * (1-cos(w_axis * time));
					posori_task->_desired_position(1) = x_init(1) - circle_radius * sin(w_circle * time);
					posori_task->_desired_position(2) = x_init(2) + circle_radius * (1-cos(w_circle * time));

					posori_task->_desired_velocity(0) =  - cylinder_length/2 * w_axis * sin(w_axis * time);
					posori_task->_desired_velocity(1) =  - circle_radius * w_circle * cos(w_circle * time);
					posori_task->_desired_velocity(2) =    circle_radius * w_circle * sin(w_circle * time);

					posori_task->_desired_acceleration(0) =  - cylinder_length/2 * w_axis *w_axis * cos(w_axis * time);
					posori_task->_desired_acceleration(1) =    circle_radius * w_circle * w_circle * sin(w_circle * time);
					posori_task->_desired_acceleration(2) =    circle_radius * w_circle * w_circle * cos(w_circle * time);

				}
				else if(vec_trajectory_types[trajectory_type] == "orientationOnly")
				{
					double w_ori = experiment_speed_orientation_oscillation_frequency(speed);
					double time = current_time - initial_time;

					double angle = orientation_oscillation_amplitude * sin(w_ori * time);
					Matrix3d R_curr = AngleAxisd(angle, orientation_rotation_axis).toRotationMatrix();

					posori_task->_desired_orientation = R_curr * R_init;
					posori_task->_desired_angular_velocity = orientation_oscillation_amplitude * w_ori * cos(w_ori * time) * orientation_rotation_axis;
					posori_task->_desired_angular_acceleration = - orientation_oscillation_amplitude * w_ori * w_ori * sin(w_ori * time) * orientation_rotation_axis;
				
				}
				else if(vec_trajectory_types[trajectory_type] == "spiralZOrientation")
				{
					double w_circle = experiment_speed_frequency_circle(speed);
					double w_axis = experiment_speed_frequency_axis_motion(speed);
					double w_ori = experiment_speed_orientation_oscillation_frequency(speed);
					double time = current_time - initial_time;

					posori_task->_desired_position(0) = x_init(0) -	circle_radius * (1-cos(w_circle * time));
					posori_task->_desired_position(1) = x_init(1) +	circle_radius * sin(w_circle * time);
					posori_task->_desired_position(2) = x_init(2) +	cylinder_length/2 *(1-cos(w_axis * time));

					posori_task->_desired_velocity(0) = - circle_radius * w_circle * sin(w_circle * time);
					posori_task->_desired_velocity(1) =   circle_radius * w_circle * cos(w_circle * time);
					posori_task->_desired_velocity(2) =   cylinder_length/2 * w_axis * sin(w_axis * time);

					posori_task->_desired_acceleration(0) = - circle_radius * w_circle * w_circle * cos(w_circle * time);
					posori_task->_desired_acceleration(1) = - circle_radius * w_circle * w_circle * sin(w_circle * time);
					posori_task->_desired_acceleration(2) =   cylinder_length/2 * w_axis * w_axis * cos(w_axis * time);

					double angle = orientation_oscillation_amplitude * sin(w_ori * time);
					Matrix3d R_curr = AngleAxisd(angle, orientation_rotation_axis).toRotationMatrix();

					posori_task->_desired_orientation = R_curr * R_init;
					posori_task->_desired_angular_velocity = orientation_oscillation_amplitude * w_ori * cos(w_ori * time) * orientation_rotation_axis;
					posori_task->_desired_angular_acceleration = - orientation_oscillation_amplitude * w_ori * w_ori * sin(w_ori * time) * orientation_rotation_axis;
				
				}
				else if(vec_trajectory_types[trajectory_type] == "spiralXOrientation")
				{
					double w_circle = experiment_speed_frequency_circle(speed);
					double w_axis = experiment_speed_frequency_axis_motion(speed);
					double w_ori = experiment_speed_orientation_oscillation_frequency(speed);
					double time = current_time - initial_time;

					posori_task->_desired_position(0) = x_init(0) - cylinder_length/2 * (1-cos(w_axis * time));
					posori_task->_desired_position(1) = x_init(1) - circle_radius * sin(w_circle * time);
					posori_task->_desired_position(2) = x_init(2) + circle_radius * (1-cos(w_circle * time));

					posori_task->_desired_velocity(0) =  - cylinder_length/2 * w_axis * sin(w_axis * time);
					posori_task->_desired_velocity(1) =  - circle_radius * w_circle * cos(w_circle * time);
					posori_task->_desired_velocity(2) =    circle_radius * w_circle * sin(w_circle * time);

					posori_task->_desired_acceleration(0) =  - cylinder_length/2 * w_axis *w_axis * cos(w_axis * time);
					posori_task->_desired_acceleration(1) =    circle_radius * w_circle * w_circle * sin(w_circle * time);
					posori_task->_desired_acceleration(2) =    circle_radius * w_circle * w_circle * cos(w_circle * time);

					double angle = orientation_oscillation_amplitude * sin(w_ori * time);
					Matrix3d R_curr = AngleAxisd(angle, orientation_rotation_axis).toRotationMatrix();

					posori_task->_desired_orientation = R_curr * R_init;
					posori_task->_desired_angular_velocity = orientation_oscillation_amplitude * w_ori * cos(w_ori * time) * orientation_rotation_axis;
					posori_task->_desired_angular_acceleration = - orientation_oscillation_amplitude * w_ori * w_ori * sin(w_ori * time) * orientation_rotation_axis;

				}

				// switch scenario
				if(experiment_countdown == 0)
				{

					data_file.close();

					cout << "finished experiment : " << experiment_number << "/" << number_experiments << endl;
					cout << "motion type : " << vec_trajectory_types[trajectory_type];
					cout << "\tspeed : " << vec_speeds[speed];
					cout << "\tcontroller type : " << vec_controller_types[controller_type];
					cout << "\tgains : " << vec_gain_values[gains_value] << endl;
					cout << endl;
					newfile = true;
					experiment_number++;
					gains_value++;
					if(gains_value == number_gains_values)
					{
						gains_value = 0;
						controller_type++;
						if(controller_type == number_controller_types)
						{
							controller_type = 0;
							speed++;

							joint_task->reInitializeTask();
							joint_task->_desired_position = initial_q;
							joint_task->_use_interpolation_flag = true;
							state = GO_TO_INITIAL;

							if(speed == number_speeds)
							{
								speed = 0;
								trajectory_type++;
							}

						}
					}
					if(trajectory_type < number_trajectory_types)
					{
						if(vec_trajectory_types[trajectory_type] == "orientationOnly")
						{
							experiment_countdown = 1000 * number_of_oscillation_per_experiment * 2 * M_PI / experiment_speed_orientation_oscillation_frequency(speed);
						}
						else
						{
							experiment_countdown = 1000 * number_of_oscillation_per_experiment * 2 * M_PI / experiment_speed_frequency_axis_motion(speed);
						}
					}

				}
				experiment_countdown--;
			}
		
			

			// compute torques
			posori_task->computeTorques(posori_task_torques);
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

	// data_file.close();

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

	return_filename += vec_trajectory_types[trajectory_type] + "_";
	return_filename += vec_speeds[speed] + "_";
	return_filename += vec_controller_types[controller_type] + "_";
	return_filename += vec_gain_values[gains_value];

	return_filename += ".txt";

	return return_filename;

}

void write_first_line(ofstream& file_handler)
{
	switch(state)
	{
		case EXPERIMENT :
			file_handler << "joint positions[7]\tjoint velocities[7]\tdesired_position[3]\tcurrent_position[3]\tdesired_velocity[3]\tcurrent_velocity[3]\t" <<
				"orientation_error[3]\tdesired_angular_velocity[3]\tcurrent_angular_velocity[3]\t" <<
				"posori_task_force[6]\tposori_task_torques[7]\tcommand_torques[7]\n";
			break;
		case DEBUG :
			file_handler << "debug_file\n";
			break;
	}

}
