// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "redis/RedisClient.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_IK.urdf";
const vector<string> robot_files = {
	"./resources/panda_arm_AllegroHand.urdf",
};
const vector<string> robot_names = {
	"PANDA_HAND",
};
const string camera_name = "camera_fixed";

const int n_robots = robot_names.size();

// redis keys:
// - read:
const string ROBOT_JOINT_ANGLES_KEY = "sai2::FrankaPanda::Clyde::sensors::q";
const string HAND_JOINT_ANGLES_KEY = "sai2::allegroHand::sensors::joint_positions";
const string DESIRED_FINGERTIP_POSITIONS_KEY = "sai2::PandaApplication::desired_fingertip_pos_in_world_frame";

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
	const string palm_link = "link7";
	const Vector3d palm_pos_in_link = Vector3d(0.05, 0.0, 0.15);
	robot->_q.head(7) << 0.212565,-0.523804,-0.154537,-2.13458,-0.0625216,1.71954,-0.0469045;


	// joint limits
	VectorXd q_min = VectorXd::Zero(dof);
	VectorXd q_max = VectorXd::Zero(dof);
	VectorXd q_weights = VectorXd::Zero(dof);

	q_min.head(7) << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
	q_max.head(7) << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
	q_weights.head(7) = VectorXd::Ones(7);
	q_weights(5) = 0.001;
	q_weights(6) = 0.001;

	q_min.tail(16) << -0.47, -0.196, -0.174, -0.227, -0.47, -0.196, -0.174, -0.227, -0.47, -0.196, -0.174, -0.227, -1.396, -0.105, -0.189, -0.162;
	q_max.tail(16) << 0.47, 1.61, 1.709, 1.618, 0.47, 1.61, 1.709, 1.618, 0.47, 1.61, 1.709, 1.618, -0.263, 1.163, 1.644, 1.719;
	q_weights.tail(16) = 1000.0*VectorXd::Ones(16);

	// cout << q_weights.transpose() << endl;


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

	if(!redis_client.exists(ROBOT_JOINT_ANGLES_KEY))
	{
		redis_client.setEigenMatrixJSON(ROBOT_JOINT_ANGLES_KEY, robot->_q.head(7));
	}
	if(!redis_client.exists(HAND_JOINT_ANGLES_KEY))
	{
		redis_client.setEigenMatrixJSON(HAND_JOINT_ANGLES_KEY, VectorXd::Zero(16));
	}
	if(!redis_client.exists(DESIRED_FINGERTIP_POSITIONS_KEY))
	{
		VectorXd tmp_des_ft_pos = VectorXd::Zero(9);
		tmp_des_ft_pos.segment<3>(0) = ik_desired_positions[0];
		tmp_des_ft_pos.segment<3>(3) = ik_desired_positions[1];
		tmp_des_ft_pos.segment<3>(6) = ik_desired_positions[2];
		redis_client.setEigenMatrixJSON(DESIRED_FINGERTIP_POSITIONS_KEY, tmp_des_ft_pos);
	}

	// read from redis the current robot configuration and contact points
	robot->_q.head(7) = redis_client.getEigenMatrixJSON(ROBOT_JOINT_ANGLES_KEY);
	robot->_q.tail(16) = redis_client.getEigenMatrixJSON(HAND_JOINT_ANGLES_KEY);
	VectorXd tmp_fingertip_desired_pos = redis_client.getEigenMatrixJSON(DESIRED_FINGERTIP_POSITIONS_KEY);
	ik_desired_positions[0] = tmp_fingertip_desired_pos.segment<3>(0);
	ik_desired_positions[1] = tmp_fingertip_desired_pos.segment<3>(3);
	ik_desired_positions[2] = tmp_fingertip_desired_pos.segment<3>(6);
	
	robot->updateKinematics();

	robot->position(desired_palm_position, palm_link, palm_pos_in_link);
	robot->rotation(desired_palm_orientation, palm_link);
	desired_finger_configuration = robot->_q.tail(16);

	// compute inverse kinematics
	VectorXd q_ik = VectorXd::Zero(dof);	

	robot->computeIK3d_JL(q_ik, ik_links, ik_pos_in_link, ik_desired_positions, q_min, q_max, q_weights);
	robot->_q = q_ik;
	robot->updateKinematics();

	// compute new palm pos and ori
	robot->position(desired_palm_position, palm_link, palm_pos_in_link);
	robot->rotation(desired_palm_orientation, palm_link);
	desired_finger_configuration = robot->_q.tail(16);
	desired_finger_configuration.segment<4>(8) << 0.0, 0.1, 0.1, 0.05;

	cout << desired_palm_orientation * desired_palm_orientation.transpose() << endl;

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

