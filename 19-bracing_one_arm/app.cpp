// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "timer/LoopTimer.h"
#include "uiforce/UIForceWidget.h"
#include "force_sensor/ForceSensorSim.h"
#include "MomentumObserver.h"
#include "Logger.h"

#include "tasks/JointTask.h"
#include "tasks/PositionTask.h"
#include "tasks/PosOriTask.h"

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

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget, ForceSensorSim* force_sensor);
unsigned long long controller_counter = 0;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

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

// flags for ui widget click
bool fRobotLinkSelect = false;
Vector3d ui_force;
VectorXd ui_force_command_torques;

// force sensor
const string link_name = "end-effector";
const Vector3d pos_in_link = Vector3d(0,0,0);
Vector3d sensed_force;
Vector3d sensed_moment;

// debug
VectorXd tau_contact_from_simulation;

const bool contact_task = true;
// const bool contact_task = false;

// const bool flag_simulation = false;
const bool flag_simulation = true;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.0);

	// load robots
	Affine3d T_world_robot = sim->getRobotBaseTransform(robot_name);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);
	robot->updateKinematics();

	// force sensor
	Affine3d T_link_oppoint = Affine3d::Identity();
	T_link_oppoint.translation() = pos_in_link;
	auto force_sensor = new ForceSensorSim(robot_name, link_name, T_link_oppoint, robot);
	force_sensor->enableFilter(0.001);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateKinematics();

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
	ui_force_widget->setEnable(false);

	ui_force_widget->_spring_k = 50.0;
	ui_force_widget->_max_force = 100.0;

	int dof = robot->dof();
	ui_force.setZero();
	ui_force_command_torques.setZero(dof);


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

	// start simulation hread
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim, ui_force_widget, force_sensor);
	thread control_thread(control, robot, sim);

	// while window is open:
	while (!glfwWindowShouldClose(window))
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

	    // detect click to the link
		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			string ret_link_name;
			Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			if(!ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix))
			{
				fRobotLinkSelect = false;
			}
		}

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Vector3d cam_lookat_axis = camera_lookat;
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
			Matrix3d m_tilt; m_tilt = AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Matrix3d m_pan; m_pan = AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	control_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim)
{
	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);
	VectorXd gravity = VectorXd::Zero(dof);
	VectorXd command_torques = VectorXd::Zero(dof);

	// prepare controller	
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	joint_task->_kp = 50.0;
	joint_task->_kv = 15.0;

	// posori controller
	Affine3d T_link_sensor = Affine3d::Identity();
	T_link_sensor.translation() = pos_in_link;
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	MatrixXd posori_task_range_space = MatrixXd::Zero(dof,dof);
	posori_task->_use_interpolation_flag = false;

	// posori_task->_kp_pos = 100.0;
	// posori_task->_kv_pos = 20.0;
	// posori_task->_kp_ori = 100.0;
	// posori_task->_kv_ori = 20.0;

	double amplitude_motion = 0.02;
	double freq_motion = 0.2;
	Vector3d x_init = posori_task->_current_position;

	posori_task->setForceSensorFrame(link_name, T_link_sensor);
	// posori_task->setClosedLoopForceControl();

	Vector3d desired_force = Vector3d(0, 0, -10.0);

	posori_task->_kp_force = 0.7;
	posori_task->_ki_force = 2.5;
	posori_task->_kv_force = 15.0;

	// momentum observer
	auto momentum_observer = new MomentumObserver(robot, 0.001);
	double gain = 15.0;
	momentum_observer->setGain(gain * MatrixXd::Identity(dof,dof));
	VectorXd tau_contact_observed = VectorXd::Zero(dof);

	// bracing task
	MatrixXd N_bracing = MatrixXd::Zero(dof,dof);
	MatrixXd range_space_bracing = MatrixXd::Zero(dof,dof);
	double alpha = 0;
	VectorXd bracing_task_torques = VectorXd::Zero(dof);

	// logger
	string folder = "../../19-bracing_one_arm/data_logging/data/";
	string timestamp = currentDateTime();
	string prefix = "data";
	string suffix = ".csv";
	string filename = folder + prefix + "_" + timestamp + suffix;
	auto logger = new Logging::Logger(10000, filename);
	
	Vector3d current_position = posori_task->_current_position;
	Vector3d desired_position = posori_task->_desired_position;

	logger->addVectorToLog(&command_torques, "command_torques");
	logger->addVectorToLog(&sensed_force, "sensed_force");
	logger->addVectorToLog(&current_position, "current_position");
	logger->addVectorToLog(&desired_position, "desired_position");

	logger->start();

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	double prev_time = 0;

	while (fSimulationRunning) 
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		double dt = time - prev_time;

		// read robot state from redis
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		// update momentum observer
		VectorXd task_contact_torques = VectorXd::Zero(dof);
		task_contact_torques = posori_task->_jacobian.block(0,0,3,dof).transpose() * sensed_force;
		momentum_observer->update(command_torques, task_contact_torques);
		tau_contact_observed = momentum_observer->getDisturbanceTorqueEstimate();
		// tau_contact_observed.tail(3) = VectorXd::Zero(3);

		// update sensed force
		posori_task->updateSensedForceAndMoment(posori_task->_current_orientation.transpose() * sensed_force, Vector3d::Zero());

		// update robot model
		robot->updateModel();
		robot->gravityVector(gravity);

		// update tasks models
		N_prec.setIdentity();

		posori_task->updateTaskModel(N_prec);
		posori_task_range_space = posori_task->_N_prec - posori_task->_N;
		N_prec = posori_task->_N;

		MatrixXd J_bracing_estimate = tau_contact_observed.transpose() * N_prec;
		MatrixXd Proj_bracing = MatrixXd::Zero(dof,dof);
		if(tau_contact_observed.norm() > 10.0)
		{
			MatrixXd Lambda_bracing = MatrixXd::Zero(1,1);
			MatrixXd Jbar_bracing = MatrixXd::Zero(dof,1);
			// robot->nullspaceMatrix(N_bracing, J_bracing_estimate, N_prec);
			robot->operationalSpaceMatrices(Lambda_bracing, Jbar_bracing, N_bracing, J_bracing_estimate, N_prec);
			range_space_bracing = N_prec - N_bracing;
			Proj_bracing = tau_contact_observed * Jbar_bracing.transpose();
			N_prec = N_bracing;
		}
		else
		{
			N_bracing.setIdentity();
			range_space_bracing.setZero();
		}

		joint_task->updateTaskModel(N_prec);

		// compute torques
		// pos task
		if(contact_task)
		{
			if(sensed_force.norm() < 1)
			{
				posori_task->_desired_position(2) -= 0.0001;
			}
			else
			{
				if(controller_counter > 2000)
				{
					posori_task->setClosedLoopForceControl();
				}
				posori_task->setForceAxis(Vector3d::UnitZ());
				posori_task->_desired_force(2) = -10.0;
			}
		}

		// motion
		double omega = 2 * M_PI * freq_motion;
		posori_task->_desired_position(0) = x_init(0) + amplitude_motion * (cos(omega*time)-1);
		posori_task->_desired_position(1) = x_init(1) + amplitude_motion * sin(omega*time);

		posori_task->_desired_velocity(0) = - amplitude_motion * omega * sin(omega*time);
		posori_task->_desired_velocity(1) = amplitude_motion * omega * cos(omega*time);

		posori_task->_desired_acceleration(0) = - amplitude_motion * omega * omega * cos(omega*time);
		posori_task->_desired_acceleration(1) = - amplitude_motion * omega * omega * sin(omega*time);

		posori_task->computeTorques(posori_task_torques);

		posori_task_torques += posori_task_range_space.transpose() * (gravity + tau_contact_observed);

		// joint task
		joint_task->computeTorques(joint_task_torques);
		if(controller_counter < 10000)
		{
			joint_task_torques += joint_task->_N_prec.transpose() * gravity;
		}
		// else if(controller_counter > 16000)
		// {
		// 	joint_task_torques += N_bracing.transpose() * (gravity + tau_contact_observed);
		// }

		// bracing task
		if(controller_counter % 10 == 0)
		{
			if(range_space_bracing.norm() > 1e-3)
			{
				VectorXd Pbg = Proj_bracing*gravity;
				double Pbg_sqnorm = Pbg.squaredNorm();
				alpha = 1 - gravity.dot(Pbg)/Pbg_sqnorm;
				if(sensed_force.norm() > 1)
				{
					alpha -= (posori_task->_jacobian.block(0,0,3,dof).transpose()*desired_force).dot(Pbg)/Pbg_sqnorm;
				}
			}
			else
			{
				alpha = 0;
			}
		}
		// if(alpha < -1)
		// {
		// 	alpha = -1;
		// }
		// if(alpha > 1)
		// {
		// 	alpha = 1;
		// }
		alpha = 0;
		bracing_task_torques = alpha * posori_task->_N.transpose() * Proj_bracing * gravity;

		// final torques
		command_torques = posori_task_torques + bracing_task_torques + joint_task_torques;

		// send to redis
		// command_torques = gravity;
		// command_torques.setZero();
		sim->setJointTorques(robot_name, command_torques);


		// logger
		current_position = posori_task->_current_position;
		desired_position = posori_task->_desired_position;


		if(controller_counter % 100 == 0)
		{
			cout << "counter : " << controller_counter << endl;
			// cout << "bracing task torques :\n" << bracing_task_torques.transpose() << endl; 
			// cout << "sensed force :\n" << sensed_force.transpose() << endl;
			// cout << "tau contact observed :\n" << tau_contact_observed.transpose() << endl;
			// cout << "tau contact from simulation :\n" << tau_contact_from_simulation.transpose() << endl;
			// cout << "task torques :\n" << posori_task_torques.transpose() << endl;
			// cout << "joint torques :\n" << joint_task_torques.transpose() << endl;
			// cout << "bracing torques :\n" << bracing_task_torques.transpose() << endl;
			// cout << "command torques :\n" << command_torques.transpose() << endl;
			
			// cout << "task contact torques :\n" << task_contact_torques.transpose() << endl; 

			// cout << "desired position :\n" << posori_task->_desired_position.transpose() << endl; 

			cout << "q : " << robot->_q.transpose() << endl;
			cout << "norm 1 control torques : " << command_torques.lpNorm<1>() << endl;
			cout << "norm 1 control torques arm only : " << command_torques.tail(7).lpNorm<1>() << endl;
			cout << "alpha : " << alpha << endl;
			cout << "gravity : " << gravity.transpose() << endl;
			cout << "norm 1 gravity : " << gravity.lpNorm<1>() << endl;
 
			// cout << endl;

			// MatrixXd Jbar_bracing_dbg = MatrixXd(dof,1);
			// robot->dynConsistentInverseJacobian(Jbar_bracing_dbg, J_bracing_estimate);
			// MatrixXd range_bracing_bis = tau_contact_observed * Jbar_bracing_dbg.transpose();

			// cout << "diff range space bracing : " << (range_space_bracing.transpose() - range_bracing_bis).norm() << endl;
			// cout << "diff range space bracing bis : " << (range_space_bracing.transpose() - posori_task->_N.transpose()*range_bracing_bis).norm() << endl;
			// cout << "diff range space bracing ter : " << (range_bracing_bis - posori_task->_N.transpose()*range_bracing_bis).norm() << endl;

			// cout << "gravity :\n" << gravity.transpose() << endl;
			// cout << "z pot point : " << z_pot_point << endl;
			// cout << "pot force : " << pot_force << endl;
			cout << endl; 
		}

		prev_time = time;
		controller_counter++;

	}

	logger->stop();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget, ForceSensorSim* force_sensor) {

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(robot->dof());

	// debug mom contact observer
	const string bracing_contact_link = "link4";
	const Vector3d bracing_contact_pos_in_link = Vector3d(-0.085, 0.065, 0);
	Affine3d T_fsensor_bracing = Affine3d::Identity();
	T_fsensor_bracing.translation() = bracing_contact_pos_in_link;
	auto force_sensor_bracing = new ForceSensorSim(robot_name, bracing_contact_link, T_fsensor_bracing, robot);
	force_sensor_bracing->enableFilter(0.005);
	MatrixXd J_bracing = MatrixXd::Zero(3,dof);
	Vector3d f_sensed_bracing = Vector3d::Zero();

	// create a timer
	double sim_frequency = 15000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_frequency); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// get ui force and torques
		if(ui_force_widget->getState() == UIForceWidget::UIForceWidgetState::Active)
		{
			ui_force_widget->getUIForce(ui_force);
			ui_force_widget->getUIJointTorques(ui_force_command_torques);
		}
		else
		{
			ui_force.setZero();
			ui_force_command_torques.setZero(dof);
		}

		// command_torques += ui_force_command_torques;

		// set torques to simulation
		// sim->setJointTorques(robot_name, command_torques);

		// integrate forward
		sim->integrate(1.0/sim_frequency);

		// update force sensor
		force_sensor->update(sim);
		force_sensor->getForce(sensed_force);
		force_sensor->getMoment(sensed_moment);

		sensed_force *= -1;
		sensed_moment *= -1;

		// debug mom observer
		force_sensor_bracing->update(sim);
		force_sensor_bracing->getForce(f_sensed_bracing);
		robot->JvWorldFrame(J_bracing, bracing_contact_link, bracing_contact_pos_in_link);
		tau_contact_from_simulation = J_bracing.transpose() * f_sensed_bracing;

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
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

