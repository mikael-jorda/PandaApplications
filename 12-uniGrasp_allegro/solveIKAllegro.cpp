// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "redis/RedisClient.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

#include <chrono>

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_IK.urdf";
const vector<string> robot_files = {
	"./resources/floatingAllegroHand.urdf",
};
const vector<string> robot_names = {
	"ALLEGRO_HAND",
};
const string camera_name = "camera_fixed";

const int n_robots = robot_names.size();

// redis keys:
// - read:
const string CAMERA_POS_IN_WORLD_KEY = "sai2::PandaApplication::camera::camera_pos_in_world";
const string CAMERA_ROT_IN_WORLD_KEY = "sai2::PandaApplication::camera::camera_rot_in_world";
const string DESIRED_FINGERTIP_POSITIONS_IN_CAMERA_FRAME_KEY = "sai2::PandaApplication::desired_fingertip_pos_in_camera_frame";

// - write
const string ROBOT_DESIRED_PALM_POSITION_KEY = "sai2::unigraspAllegro::desired_palm_position_from_IK";
const string ROBOT_DESIRED_PALM_ORIENTATION_KEY = "sai2::unigraspAllegro::desired_palm_orientation_from_IK";
const string ROBOT_DESIRED_FINGER_CONFIGURATION_KEY = "sai2::unigraspAllegro::desired_finger_configuration_from_IK";
const string IK_FINISHED_KEY = "sai2::unigraspAllegro::ik_finished";

RedisClient redis_client;
Vector3d desired_palm_position;
Matrix3d desired_palm_orientation;
VectorXd desired_finger_configuration;

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

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_files[0], false);
	int dof = robot->dof();
	const string palm_link = "palm";
	const Vector3d palm_pos_in_link = Vector3d(0,0,0);

	robot->_q(2) = 0.5;
	robot->_q(5) = 115.0/180.0*M_PI;

	// joint limits
	VectorXd q_min = VectorXd::Zero(dof);
	VectorXd q_max = VectorXd::Zero(dof);
	VectorXd q_weights = VectorXd::Ones(dof);

	q_min.head(6) = -1000.0 * VectorXd::Ones(6);
	q_max.head(6) = 1000.0 * VectorXd::Ones(6);
	// q_weights.head(6) = VectorXd::Ones(6);

	q_min.tail(16) << -0.47, -0.196, -0.174, -0.227, -0.47, -0.196, -0.174, -0.227, -0.47, -0.196, -0.174, -0.227, -1.396, -0.105, -0.189, -0.162;
	q_max.tail(16) << 0.47, 1.61, 1.709, 1.618, 0.47, 1.61, 1.709, 1.618, 0.47, 1.61, 1.709, 1.618, -0.263, 1.163, 1.644, 1.719;
	// q_weights.tail(16) = 1000.0*VectorXd::Ones(16);

	robot->_q.tail(16) = (q_min.tail(16) + q_max.tail(16))/2.0;
	robot->_q(6) = 0;
	robot->_q(10) = 0;
	robot->_q(14) = 0;
	robot->_q(19) = 0;

	// prepare IK points
	vector<string> ik_links = {
		"link_15.0_tip",
		"link_3.0_tip",
		"link_7.0_tip",
	};

	vector<Vector3d> ik_pos_in_link = {
		Vector3d::Zero(),
		Vector3d::Zero(),
		Vector3d::Zero(),
	};

	vector<Vector3d> ik_desired_positions = {
		Vector3d(0.45,0.0,0.5),
		Vector3d(0.5,0.0,0.5),
		Vector3d(0.5,-0.05,0.5),
	};

	if(!redis_client.exists(CAMERA_POS_IN_WORLD_KEY))
	{
		redis_client.setEigenMatrixJSON(CAMERA_POS_IN_WORLD_KEY, Vector3d::Zero());
	}
	if(!redis_client.exists(CAMERA_ROT_IN_WORLD_KEY))
	{
		redis_client.setEigenMatrixJSON(CAMERA_ROT_IN_WORLD_KEY, Matrix3d::Identity());
	}
	if(!redis_client.exists(DESIRED_FINGERTIP_POSITIONS_IN_CAMERA_FRAME_KEY))
	{
		VectorXd tmp_des_ft_pos = VectorXd::Zero(9);
		tmp_des_ft_pos.segment<3>(0) = ik_desired_positions[0];
		tmp_des_ft_pos.segment<3>(3) = ik_desired_positions[1];
		tmp_des_ft_pos.segment<3>(6) = ik_desired_positions[2];
		redis_client.setEigenMatrixJSON(DESIRED_FINGERTIP_POSITIONS_IN_CAMERA_FRAME_KEY, tmp_des_ft_pos);
	}

	// read transform between world and camera from redis, and the desired contact points in camera frame
	Affine3d T_world_camera = Affine3d::Identity();
	T_world_camera.translation() = redis_client.getEigenMatrixJSON(CAMERA_POS_IN_WORLD_KEY);
	T_world_camera.linear() = redis_client.getEigenMatrixJSON(CAMERA_ROT_IN_WORLD_KEY);
	VectorXd tmp_fingertip_desired_pos = redis_client.getEigenMatrixJSON(DESIRED_FINGERTIP_POSITIONS_IN_CAMERA_FRAME_KEY);

	// transform the points to world frame
	ik_desired_positions[0] = T_world_camera * tmp_fingertip_desired_pos.segment<3>(0);
	ik_desired_positions[1] = T_world_camera * tmp_fingertip_desired_pos.segment<3>(3);
	ik_desired_positions[2] = T_world_camera * tmp_fingertip_desired_pos.segment<3>(6);
	
	// find a good initialization for the position of the hand 
	// make z hand coincide with the axis from the thumb contact point to the index contact poinr
	// put the hand above the thumb contact point by 10cm
	Vector3d p_thumb_index = ik_desired_positions[1] - ik_desired_positions[0];
	Vector3d p_index_middle = ik_desired_positions[2] - ik_desired_positions[1];
	p_thumb_index.normalize();
	p_index_middle.normalize();
	double angle = atan2(p_thumb_index(1), p_thumb_index(0));
	robot->_q(3) = angle;

	robot->_q(0) = ik_desired_positions[0](0);
	robot->_q(1) = ik_desired_positions[0](1);
	robot->_q(2) = ik_desired_positions[0](2) + 0.17;
	robot->_q.head(3) += 0.05 * p_index_middle;
	robot->_q.head(3) += 0.05 * p_thumb_index;

	robot->updateKinematics();


	auto t1 = std::chrono::high_resolution_clock::now();

	// compute inverse kinematics
	VectorXd q_ik = VectorXd::Zero(dof);	
	robot->computeIK3d_JL(q_ik, ik_links, ik_pos_in_link, ik_desired_positions, q_min, q_max, q_weights);
	// robot->computeIK3d(q_ik, ik_links, ik_pos_in_link, ik_desired_positions);
	robot->_q = q_ik;
	robot->updateKinematics();

	// compute new palm pos and ori
	robot->position(desired_palm_position, palm_link, palm_pos_in_link);
	robot->rotation(desired_palm_orientation, palm_link);
	desired_finger_configuration = robot->_q.tail(16);
	desired_finger_configuration.segment<4>(8) << 0.0, 0.1, 0.1, 0.05;

	auto t2 = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

    cout << "time for IK solver in ms : " << duration/1000.0 << endl;;
	/*------- Set up visualization -------*/
	// display contact points
	vector<chai3d::cShapeSphere*> graphic_contact_points;
	for(int i=0 ; i<3 ; i++)
	{
		graphic_contact_points.push_back(new chai3d::cShapeSphere(0.015));
		graphic_contact_points[i]->m_material->setColorf(1.0,0.0,0.0);
		graphics->_world->addChild(graphic_contact_points[i]);
		graphic_contact_points[i]->setLocalPos(ik_desired_positions[i]);
	}

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

	// while window is open:
	while (!glfwWindowShouldClose(window))
	{
		// robot->_q.tail(16) = redis_client.getEigenMatrixJSON(HAND_JOINT_ANGLES_KEY);
		// robot->updateKinematics();
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_names[0], robot);
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

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
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
		case GLFW_KEY_Y:
			redis_client.setEigenMatrixJSON(ROBOT_DESIRED_PALM_POSITION_KEY, desired_palm_position);
			redis_client.setEigenMatrixJSON(ROBOT_DESIRED_PALM_ORIENTATION_KEY, desired_palm_orientation);
			redis_client.setEigenMatrixJSON(ROBOT_DESIRED_FINGER_CONFIGURATION_KEY, desired_finger_configuration);
			redis_client.set(IK_FINISHED_KEY, "1");
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

