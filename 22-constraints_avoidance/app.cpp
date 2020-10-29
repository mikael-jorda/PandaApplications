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
Vector3d f_sensed_elbow;

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
	force_sensor->enableFilter(0.0005);

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
	VectorXd coriolis_plus_gravity = VectorXd::Zero(dof);
	VectorXd command_torques = VectorXd::Zero(dof);

	// prepare controller	
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	joint_task->_kp = 50.0;
	joint_task->_kv = 12.0;
	joint_task->_ki = 0.0;

	// posori controller
	Affine3d T_link_sensor = Affine3d::Identity();
	T_link_sensor.translation() = pos_in_link;
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	posori_task->_otg->setMaxLinearVelocity(0.07);

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	// MatrixXd posori_task_range_space = MatrixXd::Zero(dof,dof);
	// posori_task->_use_interpolation_flag = false;

	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 20.0;

	posori_task->setForceSensorFrame(link_name, T_link_sensor);
	// posori_task->setClosedLoopForceControl();

	Vector3d desired_force = Vector3d(0, 0, -10.0);

	posori_task->_kp_force = 0.7;
	posori_task->_ki_force = 2.3;
	posori_task->_kv_force = 15.0;

	bool contact_made = false;

	// obstacle avoidance
	VectorXd constraint_task_torques = VectorXd::Zero(dof);
	const Vector3d obstacle_position = Vector3d(0.8, 0.1, 0.32);
	const double r_obstacle = 0.06;
	const double r_transition = 0.10;
	const double r_influence = 0.14;

	const double d_z = r_influence - r_obstacle;
	const double d_t = r_transition - r_obstacle;
	double d_c = 0;

	Vector3d u_obstacle = Vector3d::Zero();

	MatrixXd J_c = MatrixXd::Zero(1,dof);
	MatrixXd Lambda_c = MatrixXd::Zero(1,1);
	MatrixXd Jbar_c = MatrixXd::Zero(dof,1);
	MatrixXd N_c = MatrixXd::Zero(dof,dof);

	VectorXd F_c = VectorXd::Zero(1);

	VectorXd F_tp_c = VectorXd::Zero(1);
	VectorXd F_pf = VectorXd::Zero(1);
	double alpha_c = 0;

	double kp_c = 1.0;
	double kv_c = 25.0;

	// contact driven
	vector<string> link_names = {"link0","link1","link2","link3","link4","link5","link6","link7"};
	Vector3d link_center = Vector3d(-0.085, 0.1, 0.0);

	VectorXd contact_driven_torques = VectorXd::Zero(dof);
	MatrixXd J_cd_link = MatrixXd::Zero(3,dof);
	MatrixXd Jbar_cd_link = MatrixXd::Zero(dof,3);
	MatrixXd J_cd_link_reduced = MatrixXd::Zero(1,dof);
	MatrixXd Jbar_cd_link_reduced = MatrixXd::Zero(dof,1);

	MatrixXd N_cd = MatrixXd::Zero(dof,dof);
	MatrixXd L_cd = MatrixXd::Zero(dof,dof);

	Vector3d u_contact_driven = Vector3d::Zero();

	const double F_cd_max = 15.0;
	double F_cd = F_cd_max;
	double contact_driven_torque_threshold = 0.2;
	double contact_driven_torque_norm_threshold = 10.0;

	int in_contact = 0;
	const int nsteps_for_contact = 50;

	// momentum observer
	auto momentum_observer = new MomentumObserver(robot, 0.001);
	ButterworthFilter filter_gamma = ButterworthFilter(dof, 0.015);

	double gain = 25.0;
	momentum_observer->setGain(gain * MatrixXd::Identity(dof,dof));
	VectorXd gamma_raw = VectorXd::Zero(dof);
	VectorXd gamma = VectorXd::Zero(dof);

	bool constraint_active = false;
	bool prev_constraint_active = false;

	// logger
	string folder = "../../22-constraints_avoidance/data_logging/data/";
	string timestamp = currentDateTime();
	string prefix = "data";
	string suffix = ".csv";
	string filename = folder + prefix + "_" + timestamp + suffix;
	auto logger = new Logging::Logger(1000, filename);
	
	Vector3d log_desired_position = posori_task->_desired_position;
	Vector3d log_current_position = posori_task->_current_position;
	Vector3d log_sensed_force = sensed_force;
	VectorXd log_gamma = gamma;
	Vector3d log_force_elbow = f_sensed_elbow;
	VectorXd log_obstacle_avoidance_distances = VectorXd::Zero(3);
	VectorXd log_obstacle_avoidance_force = VectorXd::Zero(3);
	VectorXd log_contact_driven_command_force = VectorXd::Zero(3);

	logger->addVectorToLog(&log_desired_position, "desired_position");
	logger->addVectorToLog(&log_current_position, "current_position");
	logger->addVectorToLog(&log_sensed_force, "sensed_force_end_effector");
	logger->addVectorToLog(&log_gamma, "momentum_observer");
	logger->addVectorToLog(&log_force_elbow, "f_sensed_elbow");
	logger->addVectorToLog(&log_obstacle_avoidance_distances, "distances_obstacle_avoidance");
	logger->addVectorToLog(&log_obstacle_avoidance_force, "obstacle_avoidance_force");
	logger->addVectorToLog(&log_contact_driven_command_force, "contact_driven_command_force");

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
		// task_contact_torques = posori_task->_jacobian.block(0,0,3,dof).transpose() * posori_task->_desired_force;
		momentum_observer->update(command_torques, task_contact_torques);
		gamma_raw = momentum_observer->getDisturbanceTorqueEstimate();
		// gamma_raw.tail(3) = VectorXd::Zero(3);
		// gamma = filter_gamma.update(gamma_raw);
		gamma = (gamma_raw);




		if(gamma.norm() > contact_driven_torque_norm_threshold)
		{
			in_contact += 1;
		}
		else
		{
			in_contact -= 1;
		}

		if(in_contact > nsteps_for_contact)
		{
			in_contact = nsteps_for_contact;
		}
		if(in_contact < 0)
		{
			in_contact = 0;
		}

		// update sensed force
		posori_task->updateSensedForceAndMoment(posori_task->_current_orientation.transpose() * sensed_force, Vector3d::Zero());

		// update robot model
		robot->updateModel();
		robot->coriolisPlusGravity(coriolis_plus_gravity);

		// compute distance to obstacle
		VectorXd p_obstacle_robot = posori_task->_current_position - obstacle_position;
		p_obstacle_robot(2) = 0;
		d_c = p_obstacle_robot.norm() - r_obstacle;

		// update tasks models
		N_prec.setIdentity();

		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;

		joint_task->updateTaskModel(N_prec);

		// compute torques
		// posori task
		if(sensed_force.norm() < 1 && !contact_made)
		{
			posori_task->_desired_position(2) -= 0.0001;
		}
		else if(!contact_made)
		{
			contact_made = true;
			// if(controller_counter > 2000)
			// {
				posori_task->setClosedLoopForceControl();
			// }
			posori_task->setForceAxis(Vector3d::UnitZ());
			posori_task->_desired_force(2) = -10.0;
		}

		// motion
		if(contact_made)
		// if(controller_counter > 0)
		{
			if(controller_counter % 8000 == 0)
			{
				posori_task->_desired_position(1) += 0.4;
			}
			else if(controller_counter % 8000 == 4000)
			{
				posori_task->_desired_position(1) -= 0.4;
			}
		}
		// if(controller_counter == 0)
		// {
		// 	posori_task->_desired_position(1) -= 0.4;
		// }

		posori_task->computeTorques(posori_task_torques);

		// joint task
		joint_task->computeTorques(joint_task_torques);

		prev_constraint_active = constraint_active;
		constraint_active = false;
		// if the constraint is active
		N_prec.setIdentity();
		if(d_c <= d_z)
		{
			constraint_active = true;
			u_obstacle = p_obstacle_robot / p_obstacle_robot.norm();
			MatrixXd Jv_robot = MatrixXd::Zero(3,dof);
			robot->Jv(Jv_robot, link_name, pos_in_link);
			J_c = u_obstacle.transpose() * Jv_robot;
			robot->operationalSpaceMatrices(Lambda_c, Jbar_c, N_c, J_c, N_prec);
			N_prec = N_c;

			// obstacle avoidance torques
			F_pf(0) = kp_c / d_c/d_c * (1/d_c - 1/d_z);
			F_tp_c = Jbar_c.transpose() * (joint_task_torques + posori_task_torques);


			double alpha_c = (d_z - d_c) / (d_z - d_t);
			if(alpha_c > 1) {alpha_c = 1;}
			// alpha_c = 1;

			F_c(0) = alpha_c * F_pf(0) + (1 - alpha_c) * F_tp_c(0);

			constraint_task_torques = J_c.transpose() * (F_c - kv_c * Lambda_c * J_c * robot->_dq);

			// cout << "Fpf: " << F_pf << endl;
			// cout << "J_c: " << J_c << endl;
			// cout << "joint_task_torques: " << joint_task_torques.transpose() << endl;
			// cout << "posori_task_torques: " << posori_task_torques.transpose() << endl;
			// cout << "F_tp_c: " << F_tp_c << endl;
			// cout << "alpha: " << alpha_c << endl;
			// cout << "F_c: " << F_c << endl;
			// cout << "constraint_task_torques: " << constraint_task_torques.transpose() << endl;
			// cout << endl << endl << endl;

			// cout << d_c << endl;
		}
		else
		{
			d_c = d_z;
			u_obstacle.setZero();
			constraint_task_torques.setZero();
		}

		if(controller_counter > 3000)
		{
			// if(link_in_contact > 2)
			if(in_contact == nsteps_for_contact)
			{

				int link_in_contact = 1;
				for(int i=0 ; i<dof ; i++)
				{
					if(abs(gamma(i)) > contact_driven_torque_threshold)
					{
						link_in_contact = i;
					}
				}

				constraint_active = true;
				MatrixXd J_cd_approx = gamma.transpose() * N_prec;
				robot->nullspaceMatrix(N_cd, J_cd_approx, N_prec);

				robot->Jv(J_cd_link, link_names[link_in_contact], link_center);
				J_cd_link = J_cd_link * N_prec;
				robot->dynConsistentInverseJacobian(Jbar_cd_link, J_cd_link);

				VectorXd F_l = - Jbar_cd_link.transpose() * gamma;
				u_contact_driven = F_l / F_l.norm();
				J_cd_link_reduced = F_l.transpose() * J_cd_link / F_l.norm();
				J_cd_link_reduced = J_cd_link_reduced;
				robot->dynConsistentInverseJacobian(Jbar_cd_link_reduced, J_cd_link_reduced);

				N_prec = N_prec * N_cd;
				L_cd = MatrixXd::Identity(dof,dof) - N_cd;

				// cout << "gamma: " << gamma.transpose() << endl;

				F_cd = F_cd_max;
				VectorXd F_tp_cd = Jbar_cd_link_reduced.transpose() * (posori_task_torques + joint_task_torques + coriolis_plus_gravity);

				VectorXd prev_force_applied = Jbar_cd_link_reduced.transpose() * command_torques;


				if(controller_counter % 1 == 0)
				{
					// cout << "gamma: " << gamma.transpose() << endl;
					// cout << "tau elbow: " << tau_contact_from_simulation.transpose() << endl;
					// // cout << "link in contact: " << link_in_contact << endl;
					// // cout << "posori task torques: " << posori_task_torques.transpose() << endl;
					// // cout << "joint task torques: " << joint_task_torques.transpose() << endl;
					// // cout << "J center link:\n" << J_cd_link << endl;
					// // cout << "Jbar center link: " << Jbar_cd_link.transpose() << endl;
					// cout << "F_l: " << F_l.transpose() << endl;
					// cout << "f sensed elbow: " << f_sensed_elbow.transpose() << endl;
					// cout << "f sensed elbow norm: " << f_sensed_elbow.norm() << endl;
					// // cout << "J center link reduced: " << J_cd_link_reduced << endl;
					// // cout << "Jbar center link reduced: " << Jbar_cd_link_reduced.transpose() << endl;
					// cout << "F tp cd: " << F_tp_cd << endl;
					// cout << "prev force applied: " << prev_force_applied << endl;
					// // cout << "delta x: " << (posori_task->_current_position.transpose() - posori_task->_desired_position.transpose()) << endl;
					

					// cout << "force applied at link center: " << (Jbar_cd_link.transpose() * L_cd.transpose() * J_cd_link_reduced.transpose() * F_cd).transpose() << endl;


					// cout << endl;
				}

				if(F_tp_cd(0) < F_cd_max && -F_cd_max < F_tp_cd(0)){F_cd = F_tp_cd(0);}
				if(-F_cd_max >= F_tp_cd(0)){F_cd = -F_cd_max;}


				// contact_driven_torques = L_cd.transpose() * (J_cd_link_reduced.transpose() * F_cd - 35.0 * robot->_dq);
				// contact_driven_torques = L_cd.transpose() * (J_cd_link_reduced.transpose() * F_cd - 15.0 * robot->_dq + gamma);
				// contact_driven_torques = L_cd.transpose() * (J_cd_link_reduced.transpose() * F_cd + gamma);
				// contact_driven_torques = L_cd.transpose() * ( - 35.0 * robot->_dq);

				contact_driven_torques = L_cd.transpose() * (J_cd_link_reduced.transpose() * F_cd);

			}
		}
		else
		{
			u_contact_driven.setZero();
			contact_driven_torques.setZero();
			gamma.setZero();
		}

		// if(!constraint_active && prev_constraint_active)
		// {
		// 	VectorXd q_des = joint_task->_desired_position;
		// 	joint_task->reInitializeTask();
		// 	joint_task->_desired_position = q_des;
		// }

		// re update tasks models to be consistent with the constraint
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;

		joint_task->updateTaskModel(N_prec);

		// re compute the task torques
		posori_task->computeTorques(posori_task_torques);
		joint_task->computeTorques(joint_task_torques);


		// final torques
		command_torques = posori_task_torques + joint_task_torques + constraint_task_torques + contact_driven_torques + coriolis_plus_gravity - posori_task->_Jbar.transpose() * gamma;
		// command_torques = posori_task_torques + joint_task_torques + constraint_task_torques + contact_driven_torques + coriolis_plus_gravity - gamma;

		// send to redis
		// command_torques = coriolis_plus_gravity;
		// command_torques.setZero();
		sim->setJointTorques(robot_name, command_torques + ui_force_command_torques);


		// logger
		log_desired_position = posori_task->_desired_position;
		log_current_position = posori_task->_current_position;
		log_sensed_force = sensed_force;
		log_gamma = gamma;
		log_force_elbow = f_sensed_elbow;
		log_obstacle_avoidance_distances << d_c, d_z, d_t;
		log_obstacle_avoidance_force = F_c(0) * u_obstacle;
		log_contact_driven_command_force = F_cd * u_contact_driven;


		if(controller_counter % 50 == 0)
		{
			// cout << "controller_counter: " << controller_counter << endl;
			// cout << "gamma: " << gamma.transpose() << endl;
			// // cout << "posori_task_torques: " << posori_task_torques.transpose() << endl;
			// // cout << "dof pos task: " << posori_task->_pos_dof << endl;
			// cout << "dc: " << d_c << endl;
			// // cout << "Jc: " << J_c << endl;
			// // cout << "F_pf: " << F_pf << endl;
			// // cout << "Fc: " << F_c << endl;
			// // cout << "constraint_task_torques: " << constraint_task_torques.transpose() << endl;
			// cout << "f sensed elbow norm: " << f_sensed_elbow.norm() << endl;
			// cout << "delta x: " << (posori_task->_current_position.transpose() - posori_task->_desired_position.transpose()) << endl;
			// cout << "sensed force: " << sensed_force.transpose() << endl;
			// cout << endl << endl;
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
	// VectorXd command_torques = VectorXd::Zero(robot->dof());

	// debug mom contact observer
	const string elbow_contact_link = "link4";
	const Vector3d elbow_contact_pos_in_link = Vector3d(-0.085, 0.1, 0.0);
	Affine3d T_fsensor_elbow = Affine3d::Identity();
	T_fsensor_elbow.translation() = elbow_contact_pos_in_link;
	auto force_sensor_elbow = new ForceSensorSim(robot_name, elbow_contact_link, T_fsensor_elbow, robot);
	force_sensor_elbow->enableFilter(0.0005);
	MatrixXd J_elbow = MatrixXd::Zero(3,dof);
	f_sensed_elbow = Vector3d::Zero();

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
		force_sensor_elbow->update(sim);
		force_sensor_elbow->getForce(f_sensed_elbow);
		robot->JvWorldFrame(J_elbow, elbow_contact_link, elbow_contact_pos_in_link);
		tau_contact_from_simulation = J_elbow.transpose() * f_sensed_elbow;

		// sim->showContactInfo();

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

