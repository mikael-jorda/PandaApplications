// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"

#include <iostream>
#include <fstream>
#include <string>
#include <tinyxml2.h>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

const string bias_file_name = "../../00-force_sensor_calibration/calibration_files/Clyde_fsensor_bias.xml";

// redis keys:
// - read:
string JOINT_ANGLES_KEY;
string JOINT_VELOCITIES_KEY;
string JOINT_TORQUES_SENSED_KEY;
// - write
string JOINT_TORQUES_COMMANDED_KEY;

string FORCE_SENSED_KEY;

// - model
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string ROBOT_GRAVITY_KEY;

VectorXd readBiasXML(const string path_to_bias_file);
// void writeCalibrationXml(const string file_name, const string tool_name, const Vector3d object_com, const double object_mass);

unsigned long long controller_counter = 0;

const bool flag_simulation = false;
// const bool flag_simulation = true; 

const bool inertia_regularization = true;

int main(int argc, char** argv) 
{

	// if(argc < 3)
	// {
	//     std::cout << "Usage :\nobjectCalibration_Clyde [calibration_file_name] [tool_name]" << std::endl;
	//     return 0;
	// }
	// const string calibration_file_name_tmp = argv[1];
	// const string calibration_file_name = "../../00-force_sensor_calibration/calibration_files/" + calibration_file_name_tmp + ".xml";
	// const string tool_name = argv[2];

	if(flag_simulation)
	{
		JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

		FORCE_SENSED_KEY = "sai2::PandaApplication::sensors::force_sensor";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::Clyde::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity";		

		FORCE_SENSED_KEY = "sai2::optoforceSensor::6Dsensor::force";
	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	if(!redis_client.exists(FORCE_SENSED_KEY))
	{
		redis_client.setEigenMatrixJSON(FORCE_SENSED_KEY, VectorXd::Zero(6));
	}

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q_desired = robot->_q;
	initial_q_desired << -90, 30, 90, -90, -30, 90, 0;
	initial_q_desired = M_PI/180.0 * initial_q_desired;
	robot->updateModel();

	// prepare controller	
	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	
	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_otg->setMaxVelocity(M_PI/5);
	joint_task->_otg->setMaxAcceleration(M_PI);
	joint_task->_otg->setMaxJerk(3*M_PI);

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_ki = 300.0;
	joint_task->_kp = 400.0;
	joint_task->_kv = 25.0;

	// prepare successive positions
	VectorXd q_desired = initial_q_desired;

	vector<Vector3d> last_joint_positions_increment;

	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-90.0, -45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(45.0, 0.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, -45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, -45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(45.0, 0.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(45.0, 0.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, -45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, -45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(45.0, 0.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 45.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 45.0, 0.0));

	int measurement_number = 0;
	const int measurement_total_length = 1000;
	int measurement_counter = measurement_total_length;

	joint_task->_desired_position = q_desired;
	joint_task->_desired_position.tail(3) += last_joint_positions_increment[measurement_number]; 

	// calibration quantities
	VectorXd bias_force = readBiasXML(bias_file_name);
	VectorXd current_force_sensed = VectorXd::Zero(6);

	// return 0;

	Vector3d mean_force = Vector3d::Zero();
	Vector3d mean_moment = Vector3d::Zero();

	int n_measure_points = last_joint_positions_increment.size();
	MatrixXd local_gravities_cross = MatrixXd(3*n_measure_points, 3);
	VectorXd local_moments = VectorXd(3*n_measure_points);
	Vector3d local_gravity = Vector3d::Zero();
	Vector3d world_gravity = Vector3d(0.0, 0.0, -9.81);
	double g2 = world_gravity.transpose() * world_gravity;
	double estimated_mass = 0;
	Vector3d estimated_com = Vector3d::Zero();

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
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		current_force_sensed = redis_client.getEigenMatrixJSON(FORCE_SENSED_KEY);
		current_force_sensed -= bias_force;

		// update model
		if(flag_simulation)
		{
			robot->updateModel();
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();
		}
		joint_task->updateTaskModel(N_prec);

		// compute torques
		joint_task->computeTorques(joint_task_torques);

		command_torques = joint_task_torques;

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// cout << (joint_task->_current_position - joint_task->_desired_position).norm() << endl;

		if((joint_task->_current_position - joint_task->_desired_position).norm() < 0.015)
		{
			measurement_counter--;

			if(measurement_counter > measurement_total_length/4.0 && measurement_counter < measurement_total_length/4.0*3.0)
			{
				mean_force -= current_force_sensed.head(3);   // the driver gives the force and moment applied by the sensor to the environment
				mean_moment -= current_force_sensed.tail(3);
			}

			if(measurement_counter == 0)
			{
				Eigen::Matrix3d R;
				robot->rotation(R,"link7");

				mean_force /= (measurement_total_length/2);
				mean_moment /= (measurement_total_length/2);
				double current_mass = (double) (world_gravity.transpose() * R * mean_force) / g2;
				estimated_mass += current_mass;

				local_moments.segment<3>(3*(measurement_number)) = mean_moment;
				local_gravity = R.transpose() * world_gravity;
				local_gravities_cross.block(3*(measurement_number),0,3,3) = Sai2Model::CrossProductOperator(local_gravity);

				cout << "move to point " << measurement_number+1 << endl;

				measurement_number++;
				joint_task->_desired_position.tail(3) += last_joint_positions_increment[measurement_number]; 
				measurement_counter = measurement_total_length;
				mean_force.setZero();
				mean_moment.setZero();
			}
			if(measurement_number == n_measure_points)
			{
				cout << "bias calibration finished" << endl;
				runloop = false;
			}
		}

		controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	estimated_mass /= n_measure_points;

	MatrixXd A = - estimated_mass * local_gravities_cross;
	VectorXd b = local_moments;

	estimated_com = A.colPivHouseholderQr().solve(b);

	// writeCalibrationXml(calibration_file_name, tool_name, estimated_com, estimated_mass);

	cout << endl;
	cout << "estimated mass : " << estimated_mass << endl;
	cout << "estimated com : " << estimated_com.transpose() << endl;
	cout << endl;

	double end_time = timer.elapsedTime();
    cout << "\n";
    cout << "Controller Loop run time  : " << end_time << " seconds\n";
    cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}

// void writeCalibrationXml(const string file_name, const string tool_name, const Vector3d object_com, const double object_mass)
// {
// 	cout << "write tool properties to file " << file_name << endl;

// 	ofstream file;
// 	file.open(file_name);

// 	if(file.is_open())
// 	{
// 		file << "<?xml version=\"1.0\" ?>\n\n";
// 		file << "<tool name=\"" << tool_name << "\">\n";
// 		file << "\t<inertial>\n";
// 		file << "\t\t<origin xyz=\"" << object_com.transpose() << "\"/>\n";
// 		file << "\t\t<mass value=\"" << object_mass << "\"/>\n";
// 		file << "\t</inertial>\n";
// 		file << "</tool>" << endl;
// 		file.close();
// 	}
// 	else
// 	{
// 		cout << "could not create xml file" << endl;
// 	}
// }

VectorXd readBiasXML(const string path_to_bias_file)
{
	VectorXd sensor_bias = VectorXd::Zero(6);
	tinyxml2::XMLDocument doc;
	doc.LoadFile(path_to_bias_file.c_str());
	if (!doc.Error())
	{
		cout << "Loading bias file file ["+path_to_bias_file+"]." << endl;
		try 
		{

			std::stringstream bias( doc.FirstChildElement("force_bias")->
				Attribute("value"));
			bias >> sensor_bias(0);
			bias >> sensor_bias(1);
			bias >> sensor_bias(2);
			bias >> sensor_bias(3);
			bias >> sensor_bias(4);
			bias >> sensor_bias(5);
			std::stringstream ss; ss << sensor_bias.transpose();
			cout << "Sensor bias : "+ss.str() << endl;
		}
		catch( const std::exception& e ) // reference to the base of a polymorphic object
		{ 
			std::cout << e.what(); // information from length_error printed
			cout << "WARNING : Failed to parse bias file." << endl;
		}
	} 
	else 
	{
		cout << "WARNING : Could no load bias file ["+path_to_bias_file+"]" << endl;
		doc.PrintError();
	}
	return sensor_bias;
}