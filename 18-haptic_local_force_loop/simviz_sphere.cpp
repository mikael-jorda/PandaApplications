// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "force_sensor/ForceSensorSim.h" // Add force sensor simulation and display classes
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <string>
#include <random>
#include <queue>

#define INIT            0
#define CONTROL         1

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;
using namespace chai3d;

const string world_file = "./resources/world_sphere.urdf";
const string robot_file = "./resources/sphere.urdf";
const string robot_name = "sphere";
const string camera_name = "camera";
const string link_name = "end_effector"; //robot end-effector
// Set sensor frame transform in end-effector frame
Affine3d sensor_transform_in_link = Affine3d::Identity();
const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);
const Vector3d pos_in_link = Vector3d(0.0,0.0,0.0);

VectorXd sensed_force_moment = VectorXd::Zero(6);
VectorXd fsensor_torques = VectorXd::Zero(3);


// flags for keyboard commands
bool fMouseXp = false;
bool fMouseXn = false;
bool fMouseYp = false;
bool fMouseYn = false;
bool fMouseZp = false;
bool fMouseZn = false;
bool fMouseGripper = false;

// redis keys:
const string SPHERE_POS_KEY = "sai2::PandaApplications::18::simviz_sphere::sensors::sphere_pos";
const string SPHERE_VEL_KEY = "sai2::PandaApplications::18::simviz_sphere::sensors::sphere_vel";
const string SPHERE_COMMAND_TORQUES_KEY = "sai2::PandaApplications::18::simviz_sphere::actuators::command_torques";

const string SPHERE_SENSED_FORCE_KEY = "sai2::PandaApplications::18::simviz_sphere::sensors::sensed_force";

const string PARTICLE_POSITIONS_KEY = "sai2::PandaApplications::18::simviz_sphere::particle_positions";

RedisClient redis_client;


// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor);

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


const double coeff_friction = 0.0;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// redis client for particles only
	auto redis_client_particles = RedisClient();
	redis_client_particles.connect();

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	Affine3d T_workd_robot = Affine3d::Identity();
	T_workd_robot.translation() = Vector3d(0, 0, 0.15);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_workd_robot);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(coeff_friction);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateModel();

	// Add force sensor to the end-effector
	sensor_transform_in_link.translation() = sensor_pos_in_link;
	auto force_sensor = new ForceSensorSim(robot_name, link_name, sensor_transform_in_link, robot);
	force_sensor->enableFilter(0.07);
	auto fsensor_display = new ForceSensorDisplay(force_sensor, graphics);
	// fsensor_display->_force_line_scale = 10.0;
	fsensor_display->_force_line_scale = 0.001;

	// prepare particle filter visualization
	// particle filter parameters
	int n_particles = 70;
	MatrixXd particles_from_redis;
	particles_from_redis.setZero(3,n_particles);
	redis_client_particles.setEigenMatrixJSON(PARTICLE_POSITIONS_KEY, particles_from_redis);
	vector<Vector3d> particles;
	vector<cShapeSphere*> particles_graphics;
	const double particle_radius = 0.007;
	cColorf center_particle_color = cColorf(1.0, 0.1, 0.1, 1.0);
	cColorf sphere_particle_color = cColorf(1.0, 0.0, 0.0, 1.0);
	cColorf color_white = cColorf(1.0, 1.0, 1.0, 1.0);
	const Vector3d center_graphic_representation = Vector3d(0.0,-0.5,0.5);
	const double graphic_representation_radius = 0.15;

	// center and circle
	auto center_sphere = new chai3d::cShapeSphere(0.9*particle_radius);
	center_sphere->m_material->setColor(color_white);
	center_sphere->setLocalPos(center_graphic_representation);
	graphics->_world->addChild(center_sphere);

	auto circle_particles = new chai3d::cShapeTorus(0.0005, graphic_representation_radius);
	circle_particles->m_material->setColor(color_white);
	circle_particles->setLocalPos(center_graphic_representation);
	circle_particles->setLocalRot(AngleAxisd(M_PI/2, Vector3d::UnitY()).toRotationMatrix());
	graphics->_world->addChild(circle_particles);


	for(int i=0 ; i<n_particles ; i++)
	{
		particles.push_back(Vector3d::Zero());
		particles_graphics.push_back(new chai3d::cShapeSphere(particle_radius));
		particles_graphics[i]->m_material->setColor(center_particle_color);
		graphics->_world->addChild(particles_graphics[i]);
	}

	for(int i=0 ; i<n_particles/3 ; i++)
	{
		particles[i] = Vector3d::Random();
		particles[i].normalize();
	}

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

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim, force_sensor);

	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		particles_from_redis = redis_client_particles.getEigenMatrixJSON(PARTICLE_POSITIONS_KEY);

		for(int i=0 ; i<n_particles ; i++)
		{
			particles[i] = particles_from_redis.col(i);
		}


		for(int i=0 ; i<n_particles ; i++)
		{
			Vector3d particle_pos_circle = particles[i];
			double particle_x = particle_pos_circle(0);
			particle_pos_circle(0) = center_graphic_representation(0);
			particles_graphics[i]->setLocalPos(center_graphic_representation + graphic_representation_radius * particle_pos_circle);
			if(particles[i].norm() < 1e-3)
			{
				particles_graphics[i]->m_material->setColor(center_particle_color);
			}
			else
			{
				cColorf particle_color = sphere_particle_color;
				if(particle_x >= 0)
				{
					particle_color.m_color[1] = particle_x;
				}
				else
				{
					particle_color.m_color[2] = -particle_x;
				}

				particles_graphics[i]->m_material->setColor(particle_color);
			}
		}

		fsensor_display->update();
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
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}



//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor) {

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(SPHERE_COMMAND_TORQUES_KEY, command_torques);

	// sensed force
	Vector3d sensed_force = Vector3d::Zero();
	Vector3d sensed_moment = Vector3d::Zero();

	// contact list
	vector<Vector3d> contact_points;
	vector<Vector3d> contact_forces;

	// redis communication
	redis_client.createReadCallback(0);
	redis_client.addEigenToReadCallback(0, SPHERE_COMMAND_TORQUES_KEY, command_torques);

	redis_client.createWriteCallback(0);
	redis_client.addEigenToWriteCallback(0, SPHERE_POS_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, SPHERE_VEL_KEY, robot->_dq);
	redis_client.addEigenToWriteCallback(0, SPHERE_SENSED_FORCE_KEY, sensed_force_moment);

	// create a timer
	double sim_frequency = 5000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_frequency);
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// particle pos from controller
		redis_client.executeReadCallback(0);
		sim->setJointTorques(robot_name, command_torques);

		// integrate forward
		sim->integrate(1.0/sim_frequency);

		// get contacts for logging
		contact_points.clear();
		contact_forces.clear();
		sim->getContactList(contact_points, contact_forces, robot_name, link_name);

		// sim->showContactInfo();

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();

		// read end-effector task forces from the force sensor simulation
		force_sensor->update(sim);
		force_sensor->getForceLocalFrame(sensed_force);
		force_sensor->getMomentLocalFrame(sensed_moment);
		
		sensed_force_moment << -sensed_force, -sensed_moment;

		redis_client.executeWriteCallback(0);

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
		case GLFW_KEY_R:
			fOnOffRemote = set;
	    break;
		case GLFW_KEY_S:
			fshowCameraPose = set;
			break;
	// device input keys
			case GLFW_KEY_U:
				fMouseXp = set;
		    break;
			case GLFW_KEY_J:
				fMouseXn = set;
		    break;
			case GLFW_KEY_I:
				fMouseYp = set;
		    break;
			case GLFW_KEY_K:
				fMouseYn = set;
		    break;
			case GLFW_KEY_O:
				fMouseZp = set;
		    break;
			case GLFW_KEY_L:
				fMouseZn = set;
		    break;
			case GLFW_KEY_X:
				fMouseGripper = set;
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
