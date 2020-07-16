// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "KalmanFilter.h"
#include "filters/ButterworthFilter.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";
const string camera_name = "camera_fixed";

// redis keys:
// - write:
std::string JOINT_ANGLES_KEY  = "sai2::PandaApplication::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai2::PandaApplication::sensors::dq";
std::string JOINT_ACCELERATIONS_KEY = "sai2::PandaApplication::sensors::ddq";

std::string JOINT_ANGLES_KALMAN_KEY  = "sai2::PandaApplication::kalman_filter::q";
std::string JOINT_VELOCITIES_KALMAN_KEY = "sai2::PandaApplication::kalman_filter::dq";
std::string JOINT_ACCELERATIONS_KALMAN_KEY = "sai2::PandaApplication::kalman_filter::ddq";

std::string JOINT_VELOCITIES_FILTERED_KEY = "sai2::PandaApplication::butterworth_filter::dq";
std::string JOINT_ACCELERATIONS_FILTERED_KEY = "sai2::PandaApplication::butterworth_filter::ddq";


// - read
const std::string TORQUES_COMMANDED_KEY  = "sai2::PandaApplication::actuators::fgc";

RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

// const bool flag_simulation = false;
const bool flag_simulation = true;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	if(!flag_simulation)
	{
		JOINT_ANGLES_KEY = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
	}

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

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

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;


	if(!redis_client.exists(JOINT_ANGLES_KEY))
	{
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, VectorXd::Zero(robot->dof()-2));
	}
	if(!redis_client.exists(JOINT_VELOCITIES_KEY))
	{
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, VectorXd::Zero(robot->dof()-2));
	}

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);
	// while window is open:
	while (!glfwWindowShouldClose(window))
	{
		if(!flag_simulation)
		{
			robot->_q.head(7) = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
			robot->_dq.head(7) = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
			robot->updateKinematics();
		}

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {

	string link_name = "link7";
	Vector3d pos_in_link = Vector3d(0, 0, 0.12);
	int dof = robot->_dof;

	VectorXd command_torques = VectorXd::Zero(robot->dof());
	redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY, command_torques);

	double loop_frequency = 1000.0;

	// prepare butterworth filter
	auto filter_dq = new ButterworthFilter(dof,0.05);
	auto filter_ddq = new ButterworthFilter(dof,0.025);

	VectorXd dq_prev = VectorXd::Zero(dof);

	VectorXd dq_filtered = VectorXd::Zero(dof);
	VectorXd ddq_filtered = VectorXd::Zero(dof);

	// prepare kalman filter
	double dt = 1/loop_frequency;
	MatrixXd F = MatrixXd::Identity(21,21);
	F.block<7,7>(0,7) = dt * MatrixXd::Identity(7,7);
	F.block<7,7>(7,14) = dt * MatrixXd::Identity(7,7);

	MatrixXd H = MatrixXd::Zero(7,21);
	H.block<7,7>(0,0) = MatrixXd::Identity(7,7);

	MatrixXd Q = MatrixXd::Zero(21,21);
	Q.block<7,7>(14,14) = 1000 * MatrixXd::Identity(7,7);
 
	MatrixXd R = 0.1 * MatrixXd::Identity(7,7);

	auto kalman_filter = new KalmanFilters::KalmanFilter(1/loop_frequency, F, H, Q, R);
	VectorXd x_init = VectorXd::Zero(21);
	x_init.head(7) = robot->_q;
	kalman_filter->init(x_init);

	VectorXd q_kalman = VectorXd::Zero(7);
	VectorXd dq_kalman = VectorXd::Zero(7);
	VectorXd ddq_kalman = VectorXd::Zero(7);

	// prepare individual kalman filters
	vector<KalmanFilters::KalmanFilter*> individual_kalman_filters;
	MatrixXd F_indiv = MatrixXd::Identity(3,3);
	F_indiv(0,1) = dt;
	F_indiv(1,2) = dt;

	MatrixXd H_indiv = MatrixXd::Zero(1,3);
	H_indiv(0) = 1;

	MatrixXd Q_indiv = MatrixXd::Zero(3,3);
	Q_indiv(2,2) = 1000;

	MatrixXd R_indiv = 0.1 * MatrixXd::Identity(1,1);
	for(int i=0 ; i<dof ; i++)
	{
		individual_kalman_filters.push_back(new KalmanFilters::KalmanFilter(1/loop_frequency, F_indiv, H_indiv, Q_indiv, R_indiv));
		VectorXd x_indiv_init = VectorXd::Zero(3);
		x_indiv_init(0) = robot->_q(i);
		individual_kalman_filters[i]->init(x_indiv_init);
	}

	// setup redis
	redis_client.createWriteCallback(0);

	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToWriteCallback(0, JOINT_ACCELERATIONS_KEY, robot->_ddq);

	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KALMAN_KEY, q_kalman);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KALMAN_KEY, dq_kalman);
	redis_client.addEigenToWriteCallback(0, JOINT_ACCELERATIONS_KALMAN_KEY, ddq_kalman);

	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_FILTERED_KEY, dq_filtered);
	redis_client.addEigenToWriteCallback(0, JOINT_ACCELERATIONS_FILTERED_KEY, ddq_filtered);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(loop_frequency); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// read arm torques from redis
		command_torques = redis_client.getEigenMatrixJSON(TORQUES_COMMANDED_KEY);

		// set torques to simulation
		sim->setJointTorques(robot_name, command_torques);

		// integrate forward
		// double curr_time = timer.elapsedTime();
		// double loop_dt = curr_time - last_time; 
		sim->integrate(1.0/loop_frequency);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		sim->getJointAccelerations(robot_name, robot->_ddq);
		robot->updateKinematics();

		VectorXd vel6d = VectorXd::Zero(6);
		VectorXd accel6d = VectorXd::Zero(6);

		robot->velocity6d(vel6d, link_name, pos_in_link);
		robot->acceleration6d(accel6d, link_name, pos_in_link);

		auto start = chrono::high_resolution_clock::now();
		// update kalman filter
		// kalman_filter->update(robot->_q);
		auto duration = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();
		cout << "time kalman filter update :\n" << duration << endl;

		// start = chrono::high_resolution_clock::now();
		VectorXd y_indiv = VectorXd::Zero(1);
		for(int k=0 ; k<dof ; k++)
		{
			y_indiv(0) = robot->_q(k);
			individual_kalman_filters[k]->update(y_indiv);
			VectorXd kalman_state = individual_kalman_filters[k]->getState();
			q_kalman(k) = kalman_state(0);
			dq_kalman(k) = kalman_state(1);
			ddq_kalman(k) = kalman_state(2);
		}
		// duration = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();
		// cout << "time individual kalman filter update :\n" << duration << endl;

		// VectorXd kalman_state = kalman_filter->getState();
		// q_kalman = kalman_state.segment<7>(0);
		// dq_kalman = kalman_state.segment<7>(7);
		// ddq_kalman = kalman_state.segment<7>(14);
		
		start = chrono::high_resolution_clock::now();
		VectorXd ddq_diff = (robot->_dq - dq_prev)/dt;
		VectorXd dq_filtered_tmp = filter_dq->update(robot->_dq);
		VectorXd ddq_filtered_tmp = filter_ddq->update(ddq_diff);
		duration = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();
		cout << "time buterworth filter update :\n" << duration << endl;


		dq_filtered = dq_filtered_tmp;
		ddq_filtered = ddq_filtered_tmp;

		// if(simulation_counter % 100 == 0)
		// {
		// 	cout << "robot ddq :\n" << robot->_ddq.transpose() << endl;
		// 	cout << "velocity :\n" << vel6d.transpose() << endl;
		// 	cout << "acceleration :\n" << accel6d.transpose() << endl;
		// 	cout << endl;
		// }

		// write new robot state to redis
		redis_client.executeWriteCallback(0);

		// redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
		// redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
		// redis_client.setEigenMatrixJSON(JOINT_ACCELERATIONS_KEY, robot->_ddq);

		// redis_client.setEigenMatrixJSON(JOINT_ANGLES_KALMAN_KEY, kalman_state.segment<7>(0));
		// redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KALMAN_KEY, kalman_state.segment<7>(7));
		// redis_client.setEigenMatrixJSON(JOINT_ACCELERATIONS_KALMAN_KEY, kalman_state.segment<7>(14));

		//update last time
		// last_time = curr_time;

		dq_prev = robot->_dq;

		simulation_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

