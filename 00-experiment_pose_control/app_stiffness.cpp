// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>
#include <random>
#include <queue>
#include <fstream>

#define INIT            0
#define CONTROL         1

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";
const string camera_name = "camera";
const string link_name = "link7"; //robot end-effector
// Set sensor frame transform in end-effector frame
Affine3d sensor_transform_in_link = Affine3d::Identity();
const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);
const Vector3d pos_in_link = Vector3d(0.0,0.0,0.107);


const vector<string> vec_controller_types = {
	"fullDynDecoupling",
	"partialDynDecoupling",
	"noDynDecoupling",
	"inertiaSaturation",
};

const vector<string> vec_gain_values = {
	"lowGains",
	// "mediumGains",
	// "highGains",
};

const double fdis = 15.0;
const double mdis = 0.5;

const int number_controller_types = vec_controller_types.size();
const int number_gains_values = vec_gain_values.size();

int state = INIT;
int controller_type = 0;
int gains_value = 0;

int experiment_number = 1;
const int number_experiments = number_gains_values * number_controller_types;

VectorXd command_torques;
VectorXd disturbance_torques;

const string prefix_path = "../../00-experiment_pose_control/data_files/data/exp_stiffness_";
string create_filename();
void write_first_line(ofstream& file_handler);


// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransYp = false;
bool fTransXn = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fshowCameraPose = false;
bool fRotPanTilt = false;
// flag for enabling/disabling remote task
bool fOnOffRemote = false;

unsigned long long controller_counter = 0;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	Affine3d T_workd_robot = Affine3d::Identity();
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_workd_robot);
	command_torques = VectorXd::Zero(robot->dof());
	disturbance_torques = VectorXd::Zero(robot->dof());

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateModel();

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
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Sigma7Applications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	runloop = true;
	thread sim_thread(simulation, robot, sim);
	thread control_thread(control, robot, sim);

	// while window is open:
	while (!glfwWindowShouldClose(window) && runloop)
	{
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
			camera_pos = camera_pos + 0.01*cam_roll_axis;
			camera_lookat = camera_lookat + 0.01*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.01*cam_roll_axis;
			camera_lookat = camera_lookat - 0.01*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.01*cam_up_axis;
			camera_lookat = camera_lookat + 0.01*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.01*cam_up_axis;
			camera_lookat = camera_lookat - 0.01*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.05*cam_depth_axis;
			camera_lookat = camera_lookat + 0.05*cam_depth_axis;
		}
		if (fTransZn) {
			camera_pos = camera_pos - 0.05*cam_depth_axis;
			camera_lookat = camera_lookat - 0.05*cam_depth_axis;
		}
		if (fshowCameraPose) {
			cout << endl;
			cout << "camera position : " << camera_pos.transpose() << endl;
			cout << "camera lookat : " << camera_lookat.transpose() << endl;
			cout << endl;
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
	runloop = false;
	sim_thread.join();
	control_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}



void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {

	// setup experiment parameters
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

	// prepare controller	
	int dof = robot->dof();
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	VectorXd disturbance_force = VectorXd::Zero(6);
	MatrixXd J_disturbance = MatrixXd::Zero(6,dof);

	VectorXd initial_q = VectorXd::Zero(dof);
	// initial_q << 0.0, -45.0, 0.0, -125.0, 0.0, 80.0, 0.0;
	initial_q << 0.0, 25.0, 0.0, -115.0, 0.0, 140.0, 0.0;
	initial_q *= M_PI/180.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = true;
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

	// prepare file to write data
	bool newfile = true;
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

		// read robot state from sim
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		// update robot model
		robot->updateModel();
		robot->coriolisForce(coriolis);

		if(state == INIT)
		{
			joint_task->updateTaskModel(MatrixXd::Identity(dof,dof));

			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques + coriolis;

			if((joint_task->_desired_position - joint_task->_current_position).norm() < 1e-1)
			{
				posori_task->reInitializeTask();
				x_init = posori_task->_current_position;
				R_init = posori_task->_current_orientation;

				joint_task->_ki = 0;
				joint_task->_use_interpolation_flag = false;

				state = CONTROL;
				initial_time = current_time;
			}
		}

		else if(state == CONTROL)
		{
			// set gains
			posori_task->_kp_pos = kp_experiments(gains_value);
			posori_task->_kv_pos = kv_experiments(gains_value);
			posori_task->_kp_ori = kp_experiments(gains_value);
			posori_task->_kv_ori = kv_experiments(gains_value);
		
			// set controller type
			if(vec_controller_types[controller_type] == "noDynDecoupling")
			{
				posori_task->_kp_pos = kp_experiments(gains_value) * 7;
				posori_task->_kv_pos = kv_experiments(gains_value) * 9;
				posori_task->_kp_ori = kp_experiments(gains_value) * 0.10;
				posori_task->_kv_ori = kv_experiments(gains_value) * 0.18;

				posori_task->setDynamicDecouplingNone();
			}
			else if(vec_controller_types[controller_type] == "fullDynDecoupling")
			{
				posori_task->setDynamicDecouplingFull();
			}
			else if(vec_controller_types[controller_type] == "partialDynDecoupling")
			{
				posori_task->_kp_ori = kp_experiments(gains_value) * 0.10;
				posori_task->_kv_ori = kv_experiments(gains_value) * 0.18;

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



			disturbance_force.setZero();
			if(((current_time - initial_time) > 2.0) && ((current_time - initial_time) < 3.0))
			{
				disturbance_force(0) = fdis;
			}
			else if(((current_time - initial_time) > 5.0) && ((current_time - initial_time) < 6.0))
			{
				disturbance_force(1) = fdis;
			}
			else if(((current_time - initial_time) > 8.0) && ((current_time - initial_time) < 9.0))
			{
				disturbance_force(2) = fdis;
			}
			else if(((current_time - initial_time) > 11.0) && ((current_time - initial_time) < 12.0))
			{
				disturbance_force(3) = mdis;
			}
			else if(((current_time - initial_time) > 14.0) && ((current_time - initial_time) < 15.0))
			{
				disturbance_force(4) = mdis;
			}
			else if(((current_time - initial_time) > 17.0) && ((current_time - initial_time) < 18.0))
			{
				disturbance_force(5) = mdis;
			}

			robot->J_0(J_disturbance, link_name);
			disturbance_torques = J_disturbance.transpose() * disturbance_force;

			// if(controller_counter % 100 == 0)
			// {
			// 	cout << "controller counter : " << controller_counter << endl;
			// 	cout << "time : " << (current_time - initial_time) << endl;
			// 	cout << "difturbance force :\n" << disturbance_force.transpose() << endl;
			// 	cout << endl;
			// }

			if(newfile)
			{
				data_file.open(create_filename());
				write_first_line(data_file);
				newfile = false;
			}

			data_file << robot->_q.transpose() << '\t' << robot->_dq.transpose() << '\t' << posori_task->_desired_position.transpose() << '\t' << posori_task->_current_position.transpose() << '\t' << 
				posori_task->_desired_velocity.transpose() << '\t' << posori_task->_current_velocity.transpose() << '\t' <<
				posori_task->_orientation_error.transpose() << '\t' << posori_task->_desired_angular_velocity.transpose() << '\t' <<
				posori_task->_current_angular_velocity.transpose() << '\t' << disturbance_force.transpose() << '\t' <<
				posori_task->_task_force.transpose() << '\t' << posori_task_torques.transpose() << command_torques.transpose() <<
				endl;

			// switch scenario
			if((current_time - initial_time) > 20.0)
			{
				data_file.close();

				cout << "finished experiment : " << experiment_number << "/" << number_experiments << endl;
				cout << "controller type : " << vec_controller_types[controller_type];
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
						break;
					}
				}
				initial_time = current_time;
			}
		
			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques + coriolis;
		}

		// command_torques.setZero(dof);

		prev_time = current_time;
		controller_counter++;
	}

	command_torques.setZero(dof);
	runloop = false;

	// data_file.close();

	double end_time = timer.elapsedTime();
	cout << "\n";
	cout << "Controller Loop run time  : " << end_time << " seconds\n";
	cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}




//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {

	int dof = robot->dof();

	// create a timer
	double sim_frequency = 5000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_frequency);
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (runloop) {
		fTimerDidSleep = timer.waitForNextLoop();

		// robot->updateKinematics();

		// set joint torques
		sim->setJointTorques(robot_name, command_torques + disturbance_torques);

		// integrate forward
		sim->integrate(1.0/sim_frequency);

		// read joint positions, velocities, update model
		// sim->getJointPositions(robot_name, robot->_q);
		// sim->getJointVelocities(robot_name, robot->_dq);
		// robot->updateKinematics();

		simulation_counter++;
	}

	double end_time = timer.elapsedTime();
	cout << "\n";
	cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}


string create_filename()
{
	cout << "create filename" << endl;
	cout << "controller type : " << controller_type << endl;
	cout << "gains value : " << gains_value << endl;
	cout << endl;

	string return_filename = prefix_path;

	return_filename += vec_controller_types[controller_type] + "_";
	return_filename += vec_gain_values[gains_value];

	return_filename += ".txt";

	return return_filename;

}

void write_first_line(ofstream& file_handler)
{
	file_handler << "joint positions[7]\tjoint velocities[7]\tdesired_position[3]\tcurrent_position[3]\tdesired_velocity[3]\tcurrent_velocity[3]\t" <<
		"orientation_error[3]\tdesired_angular_velocity[3]\tcurrent_angular_velocity[3]\tdisturbance_force[6]" <<
		"posori_task_force[6]\tposori_task_torques[7]\tcommand_torques[7]\n";
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
		case GLFW_KEY_R:
			fOnOffRemote = set;
	    break;
		case GLFW_KEY_S:
			fshowCameraPose = set;
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
