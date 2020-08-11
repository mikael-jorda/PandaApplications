// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PositionTask.h"
#include "haptic_tasks/HapticController.h"
#include "filters/ButterworthFilter.h"
#include "Logger.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "force_sensor/ForceSensorSim.h" // Add force sensor simulation and display classes
#include "force_sensor/ForceSensorDisplay.h"
#include "ForceSpaceParticleFilter_weight_mem.h"

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


// Define parameters for control with keyboard
Vector3d current_mouse_position = Eigen::Vector3d(0.0, 0.01, 0.0);
int mouse_gripper = 0;
// flags for keyboard commands
bool fMouseXp = false;
bool fMouseXn = false;
bool fMouseYp = false;
bool fMouseYn = false;
bool fMouseZp = false;
bool fMouseZn = false;
bool fMouseGripper = false;

// redis keys:
//// Haptic device related keys ////
// Maximum stiffness, damping and force specifications
vector<string> DEVICE_MAX_STIFFNESS_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_stiffness",
	"sai2::ChaiHapticDevice::device1::specifications::max_stiffness",
	};
vector<string> DEVICE_MAX_DAMPING_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_damping",
	"sai2::ChaiHapticDevice::device1::specifications::max_damping",
	};
vector<string> DEVICE_MAX_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_force",
	"sai2::ChaiHapticDevice::device1::specifications::max_force",
	};
// Set force and torque feedback of the haptic device
vector<string> DEVICE_COMMANDED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force",
	};
vector<string> DEVICE_COMMANDED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_torque",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_torque",
	};
vector<string> DEVICE_COMMANDED_GRIPPER_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force_gripper",
	};
// Haptic device current position and rotation
vector<string> DEVICE_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position",
	"sai2::ChaiHapticDevice::device1::sensors::current_position",
	};
vector<string> DEVICE_ROTATION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rotation",
	"sai2::ChaiHapticDevice::device1::sensors::current_rotation",
	};
vector<string> DEVICE_GRIPPER_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position_gripper",
	"sai2::ChaiHapticDevice::device1::sensors::current_position_gripper",
	};
// Haptic device current velocity
vector<string> DEVICE_TRANS_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_trans_velocity",
	};
vector<string> DEVICE_ROT_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_rot_velocity",
	};
vector<string> DEVICE_GRIPPER_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_gripper_velocity",
	};
vector<string> DEVICE_SENSED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_force",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_force",
	};
vector<string> DEVICE_SENSED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_torque",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_torque",
	};

RedisClient redis_client;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor);
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void particle_filter();
void communication();

Vector3d delayed_robot_position = Vector3d::Zero();
Vector3d delayed_haptic_position = Vector3d::Zero();
Vector3d delayed_haptic_velocity = Vector3d::Zero();
Vector3d delayed_sensed_force = Vector3d::Zero();
Matrix3d delayed_sigma_force = Matrix3d::Zero();

Vector3d robot_position_global = Vector3d::Zero();
Vector3d haptic_position_global = Vector3d::Zero();
Vector3d haptic_velocity_global = Vector3d::Zero();
Vector3d sensed_force_global = Vector3d::Zero();
Matrix3d sigma_force_global = Matrix3d::Zero();
Matrix3d sigma_motion_global = Matrix3d::Identity();

Vector3d motion_control_pfilter;
Vector3d force_control_pfilter;
Vector3d measured_velocity_pfilter;
Vector3d measured_force_pfilter;

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

// particle filter parameters
const int n_particles = 70;
vector<Vector3d> particles;
vector<cShapeSphere*> particles_graphics;
const double particle_radius = 0.007;
cColorf center_particle_color = cColorf(1.0, 0.1, 0.1, 1.0);
cColorf sphere_particle_color = cColorf(1.0, 0.0, 0.0, 1.0);
cColorf color_white = cColorf(1.0, 1.0, 1.0, 1.0);
const Vector3d center_graphic_representation = Vector3d(0.0,-0.5,0.5);
const double graphic_representation_radius = 0.15;

// const double percent_chance_contact_appears = 0.01;
const double percent_chance_contact_disapears = 0.95;
const double mean_scatter = 0.0;
const double std_scatter = 0.005;

const double coeff_friction = 0.0;

int force_space_dimension = 0;
int previous_force_space_dimension = 0;
Vector3d force_axis = Vector3d::Zero();
Vector3d motion_axis = Vector3d::Zero();

bool adding_contact = false;
bool removing_contact = false;
const int contact_transition_steps = 200;
int contact_transition_current_step = 0;


Matrix3d sigma_force;


// logger
Vector3d log_robot_position = Vector3d::Zero();
Vector3d log_haptic_position = Vector3d::Zero();
Vector3d log_robot_force = Vector3d::Zero();
Vector3d log_haptic_force = Vector3d::Zero();
Vector3d log_sensed_force = Vector3d::Zero();
Vector3d log_eigenvalues = Vector3d::Zero();
Vector3d log_eigenvector_0 = Vector3d::Zero();
Vector3d log_eigenvector_1 = Vector3d::Zero();
Vector3d log_eigenvector_2 = Vector3d::Zero();

Vector3d log_force_axis = Vector3d::Zero();
Vector3d log_motion_axis = Vector3d::Zero();
Vector3d log_contact_0 = Vector3d::Zero();
Vector3d log_contact_1 = Vector3d::Zero();
Vector3d log_contact_2 = Vector3d::Zero();

VectorXd log_commfreq_delay_forcespacedim = VectorXd::Zero(4);


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


double sampleNormalDistribution(const double mean, const double std)
{
	// random device class instance, source of 'true' randomness for initializing random seed
    random_device rd; 
    // Mersenne twister PRNG, initialized with seed from random device instance
    mt19937 gen(rd()); 
    // instance of class normal_distribution with specific mean and stddev
    normal_distribution<float> d(mean, std); 
    // get random number with normal distribution using gen as random source
    return d(gen); 
}

double sampleUniformDistribution(const double min, const double max)
{
	double min_internal = min;
	double max_internal = max;
	if(min > max)
	{
		min_internal = max;
		max_internal = min;
	}
	// random device class instance, source of 'true' randomness for initializing random seed
    random_device rd; 
    // Mersenne twister PRNG, initialized with seed from random device instance
    mt19937 gen(rd()); 
    // instance of class uniform_distribution with specific min and max
    uniform_real_distribution<float> d(min_internal, max_internal); 
    // get random number with normal distribution using gen as random source
    return d(gen); 
}

vector<pair<Vector3d, double>> particleFilterMotionUpdate(const vector<Vector3d> particles, 
														const Vector3d motion_control, 
														const Vector3d force_control, 
														const Vector3d measured_velocity, 
														const Vector3d measured_force,
														const int contact_space_dimension)
{
	int n = particles.size();
	Vector3d motion_control_normalized = Vector3d::Zero();
	Vector3d force_control_normalized = Vector3d::Zero();
	Vector3d measured_velocity_normalized = Vector3d::Zero();
	Vector3d measured_force_normalized = Vector3d::Zero();

	// cout << "particles size : " << particles.size() << endl;

	// cout << "force control :\n" << force_control.transpose() << endl;
	// cout << "motion control :\n" << motion_control.transpose() << endl;
	// cout << endl;


	if(motion_control.norm() > 0.5)
	{
		motion_control_normalized = motion_control/motion_control.norm();
	}
	if(force_control.norm() > 0.5)
	{
		force_control_normalized = force_control/force_control.norm();
	}
	if(measured_velocity.norm() > 1e-3)
	{
		measured_velocity_normalized = measured_velocity/measured_velocity.norm();
	}
	// Matrix3d P_orth_vel = Matrix3d::Identity() - measured_velocity_normalized*measured_velocity_normalized.transpose();
	// Vector3d projected_measured_force = P_orth_vel * measured_force;
	Vector3d projected_measured_force = measured_force;
	if(projected_measured_force.norm() > 0.5)
	{
		measured_force_normalized = projected_measured_force/projected_measured_force.norm();
	}
	vector<Vector3d> augmented_particles = particles;

	int n_added_particles_center = 1;
	for(int i=0 ; i<n_added_particles_center ; i++)
	{
		augmented_particles.push_back(Vector3d::Zero());
	}

	double prob_add_particle = (1 - abs(tanh(5.0*measured_velocity.dot(motion_control_normalized)))) * (tanh(motion_control_normalized.dot(projected_measured_force)));
	// prob_add_particle *= tanh(motion_control.norm() - coeff_friction * force_control.norm());
	Matrix3d proj_motion_control_space = motion_control_normalized*motion_control_normalized.transpose();
	prob_add_particle *= tanh((proj_motion_control_space*measured_force).norm() - force_space_dimension * 1.5 * coeff_friction * (measured_force - proj_motion_control_space*measured_force).norm());


	// cout << "prob add particle : " << prob_add_particle << endl;
	// cout << "sensed vel : " << measured_velocity.transpose() << endl;
	// cout << "mot control : " << motion_control.transpose() << endl;
	// cout << "vel dot mot command : " << motion_control_normalized.dot(measured_velocity_normalized) << endl;


	// if((proj_motion_control_space*measured_force).norm() < coeff_friction * (measured_force - proj_motion_control_space*measured_force).norm())
	// {
	// 	prob_add_particle = 0;
	// }
	if(prob_add_particle < 0)
	{
		prob_add_particle = 0;
	}
	int n_added_particles = 1.5 * prob_add_particle * n;
	// int n_added_particles = n/2;
	// for(int i=0 ; i<n_added_particles ; i++)
	// {
	// 	Vector3d new_particle = motion_control_normalized;
	// 	for(int j=0 ; j<3 ; j++)
	// 	{
	// 		new_particle(j) += sampleNormalDistribution(0, 0.1);
	// 	}
	// 	new_particle.normalize();
	// 	augmented_particles.push_back(new_particle);
	// }

	for(int i=0 ; i<n_added_particles ; i++)
	{
		double alpha = (double) (i + 0.5) / (double)n_added_particles;
		Vector3d new_particle = (1 - alpha) * motion_control_normalized + alpha * force_control_normalized;
		// for(int j=0 ; j<3 ; j++)
		// {
		// 	new_particle(j) += sampleNormalDistribution(0, 0.1);
		// }
		new_particle.normalize();
		augmented_particles.push_back(new_particle);
	}


	augmented_particles.push_back(force_control_normalized);

	int n_new_particles = n_added_particles_center + n_added_particles;
	// augmented_particles.push_back(force_control_normalized);

	// cout << "motion control : " << motion_control.transpose() << endl;
	// cout << "force control : " << force_control.transpose() << endl;
	// cout << endl;

	vector<pair<Vector3d, double>> weighted_particles;
	double cumulative_weight = 0;

	// cout << "n new particles : " << augmented_particles.size() << endl;

	for(int i=0 ; i<n + n_new_particles ; i++)
	{
		// control update : compute new particle location
		Vector3d current_particle = augmented_particles[i];
		double uniform_rand = sampleUniformDistribution(0, 1);

		if(current_particle.norm() > 1e-3) // contact
		{
			// if(uniform_rand < - percent_chance_contact_disapears * current_particle.dot(force_control_normalized) ||
			// 	uniform_rand < - percent_chance_contact_disapears * current_particle.dot(motion_control_normalized))
			// {
			// 	current_particle.setZero();
			// }
			// else
			// {
				double normal_rand_1 = sampleNormalDistribution(mean_scatter, std_scatter);
				double normal_rand_2 = sampleNormalDistribution(mean_scatter, std_scatter);
				double normal_rand_3 = sampleNormalDistribution(mean_scatter, std_scatter);
				current_particle += Vector3d(normal_rand_1, normal_rand_2, normal_rand_3);

				// double normal_rand_vel = sampleNormalDistribution(mean_scatter, 5*std_scatter);
				// current_particle += normal_rand_vel * measured_velocity_normalized;

				current_particle.normalize();

			// }

				// // project on space orthogonal to velocity
				// Matrix3d P_orth_vel = Matrix3d::Identity() - measured_velocity_normalized*measured_velocity_normalized.transpose();
				// Vector3d projected_particle = P_orth_vel * current_particle;
				// current_particle = projected_particle.normalized();
		}


		// measurement update : compute weight due to force measurement
		double weight_force = 0;
		if(current_particle.norm() < 1e-3)
		{
			weight_force = 1-tanh(projected_measured_force.norm()/2);
		}
		else
		{
			// weight_force = tanh(current_particle.dot(projected_measured_force));
			// weight_force = 1.0 * tanh(current_particle.dot(projected_measured_force));
			weight_force = 1.3 * tanh(current_particle.dot(projected_measured_force));
			// weight_force = 1.5 * current_particle.dot(measured_force_normalized);

			// Matrix3d proj_particle_space = current_particle*current_particle.transpose();
			// weight_force *= tanh((proj_particle_space*measured_force).norm() - coeff_friction * (measured_force - proj_particle_space*measured_force).norm());
		}
		if(weight_force < 0)
		{
			weight_force = 0;
		}
		if(weight_force > 1)
		{
			weight_force = 1;
		}

		// measurement update : compute weight due to velocity measurement
		// double weight_velocity = 1;
		// double weight_velocity = 1 - abs(measured_velocity_normalized.dot(current_particle));
		double weight_velocity = 1 - abs(tanh(5.0*measured_velocity.dot(current_particle)));

		// final weight
		double weight = weight_force * weight_velocity;
		cumulative_weight += weight;

		// if(force_space_dimension == 1)
		// {
		// 	cout << "particle number : " << i << endl;
		// 	cout << "current particle :\n" << current_particle.transpose() << endl;
		// 	cout << "measured force : " << projected_measured_force.transpose() << endl;
		// 	cout << "measured force normalized : " << measured_force_normalized.transpose() << endl;
		// 	cout << "measured velocity : " << measured_velocity.transpose() << endl;
		// 	cout << "measured velocity normalized : " << measured_velocity_normalized.transpose() << endl;
		// 	cout << "weight force : " << weight_force << endl;
		// 	cout << "weight velocity : " << weight_velocity << endl;
		// 	cout << endl;
		// }
		// else
		// {
		// 	cout << "****" << endl;
		// }

		// if(current_particle.norm() > 1e-3 && weight > 0)
		// {
		// 	cout << "particle : " << current_particle.transpose() << endl;
		// 	cout << "measured force : " << projected_measured_force.transpose() << endl;
		// 	cout << "measured force normalized : " << measured_force_normalized.transpose() << endl;
		// 	cout << "measured velocity : " << measured_velocity.transpose() << endl;
		// 	cout << "measured velocity normalized : " << measured_velocity_normalized.transpose() << endl;
		// 	cout << "weight force : " << weight_force << endl;
		// 	cout << "weight velocity : " << weight_velocity << endl;
		// 	cout << endl;
		// }

		// if(current_particle.dot(Vector3d::UnitZ()) < -0.95)
		// {
		// 	cout << "particle index : " << i << endl;
		// 	cout << "particle : " << current_particle.transpose() << endl;
		// 	cout << "measured force : " << projected_measured_force.transpose() << endl;
		// 	cout << "measured velocity : " << measured_velocity.transpose() << endl;
		// 	cout << "weight force : " << weight_force << endl;
		// 	cout << "weight velocity : " << weight_velocity << endl;
		// 	cout << endl;
		// }

		weighted_particles.push_back(make_pair(current_particle, cumulative_weight));
	}

	// if(previous_force_space_dimension == 1 && force_space_dimension == 0)
	// {
		// cout << endl << endl << endl;
	// }


	for(int i=0 ; i<n + n_new_particles ; i++)
	{
		weighted_particles[i].second /= cumulative_weight;
	}

	return weighted_particles;

}


vector<Vector3d> particleFilterResampling(const vector<pair<Vector3d, double>> weighted_particles, const int n_particles_to_resample)
{
	vector<Vector3d> new_particles;
	int n_weighted_particles = weighted_particles.size();

	// cout << "Resampling ******--------" << endl;
	// cout << "n weighted particles : " << n_weighted_particles << endl;
	// cout << "n particles to resample : " << n_particles_to_resample << endl;
	// cout << "total weights : " << weighted_particles[n_weighted_particles-1].second << endl;

	for(int i=0 ; i<n_particles_to_resample ; i++)
	{

		int k = 0;
		double rand = sampleUniformDistribution(0,1);
		while(weighted_particles[k].second < rand && k < n_weighted_particles)
		{
			k++;
		}

		// cout << "---" << endl;
		// cout << "rand : " << rand << endl;
		// cout << "particle selected : " << k << endl;
		// cout << "particle position : " << weighted_particles[k].first.transpose() << endl;

		new_particles.push_back(weighted_particles[k].first);
	}

	// cout << endl;

	return new_particles;
}

vector<pair<Vector3d,double>> particleFilterResamplingLowVariance(const vector<pair<Vector3d, double>> weighted_particles, const int n_particles_to_resample)
{
	vector<pair<Vector3d,double>> new_particles_with_weights;
	int n_weighted_particles = weighted_particles.size();

	double n_inv = 1.0/(double)n_particles_to_resample;
	double r = sampleUniformDistribution(0,n_inv);
	int k = 0;


	// cout << "Resampling ******--------" << endl;
	// cout << "n weighted particles : " << n_weighted_particles << endl;
	// cout << "n particles to resample : " << n_particles_to_resample << endl;
	// cout << "total weights : " << weighted_particles[n_weighted_particles-1].second << endl;

	for(int i=0 ; i<n_particles_to_resample ; i++)
	{
		// cout << weighted_particles.size() << endl;
		while(r > weighted_particles[k].second)
		{
			k++;
		}
		double cum_weight_prev = 0;
		if(k > 0)
		{
			cum_weight_prev = weighted_particles[k-1].second;
		}
		new_particles_with_weights.push_back(make_pair(weighted_particles[k].first, weighted_particles[k].second - cum_weight_prev));
		r += n_inv;
	}

	// cout << "Resampling ******--------" << endl;
	// cout << "n particles : " << new_particles_with_weights.size() << endl;
	// cout << endl;

	return new_particles_with_weights;
}

vector<Vector3d> particleFilterResamplingAfterPCA(const vector<pair<Vector3d, double>> new_particles_with_weights, const int n_particles_to_resample, const Matrix3d eigen_vectors)
{
	vector<Vector3d> new_particles;

	VectorXd cluster_average_weight = VectorXd::Zero(4);
	VectorXd num_particle_in_cluster = VectorXd::Zero(4);

	for(int i=0 ; i<new_particles_with_weights.size() ; i++)
	{
		Vector3d current_particle = new_particles_with_weights[i].first;
		double dist = current_particle.norm();
		int current_cluster = 0;
		for(int k=0 ; k < 3 ; k++)
		{
			if( (current_particle - eigen_vectors.col(k)).norm() < dist )
			{
				current_cluster = k+1;
				dist = (current_particle - eigen_vectors.col(k)).norm();
			}
		}
		cluster_average_weight(current_cluster) += new_particles_with_weights[i].second;
		num_particle_in_cluster(current_cluster) += 1;
	}

	for(int k=0 ; k<4 ; k++)
	{
		if(num_particle_in_cluster(k) > 0)
		{
			cluster_average_weight(k) /= num_particle_in_cluster(k);
		}
	}


	// cout << "cluster number part :\n" << num_particle_in_cluster.transpose() << endl;
	// cout << "cluster average weight before norm :\n" << cluster_average_weight.transpose() << endl;

	cluster_average_weight /= cluster_average_weight.sum();

	// cout << "cluster average weight after norm :\n" << cluster_average_weight.transpose() << endl;
	// cout << endl;

	for(int i=0 ; i<n_particles ; i++)
	{
		new_particles.push_back(new_particles_with_weights[i].first);
	}

	return new_particles;
}

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;



	// for(int i = 0 ; i<50 ; i++)
	// {
	// 	cout << sampleUniformDistribution(0,0.5) << endl;
	// }

	// exit(0);


	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

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
	thread control_thread(control, robot, sim);
	thread particle_filter_thread(particle_filter);
	thread communication_thread(communication);

	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
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
	control_thread.join();
	particle_filter_thread.join();
	communication_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim)
{
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	int state = INIT;
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	Vector3d x_init = joint_task->_current_position;
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = false;

	// position task
	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);
	VectorXd pos_task_torques = VectorXd::Zero(dof);
	pos_task->_use_interpolation_flag = false;
	pos_task->_use_velocity_saturation_flag = false;

	pos_task->_kp = 100.0;
	pos_task->_kv = 20.0;

	double k_vir_robot = 300.0;
	const double k_vir_haptic_goal = 300.0;
	double k_vir_haptic = k_vir_haptic_goal;
	// Matrix3d k_vir_haptic = k_vir_haptic_goal * Matrix3d::Ones();
	Vector3d robot_pos_error = Vector3d::Zero();
	Vector3d haptic_pos_error = Vector3d::Zero();

	double haptic_PO = 0;

	const double max_force_diff_robot = 0.05;
	const double max_force_diff_haptic = 0.05;
	Vector3d prev_force_command_robot = Vector3d::Zero();
	Vector3d prev_force_command_haptic = Vector3d::Zero();

	auto filter_force_command_robot = new ButterworthFilter(3,0.05);
	auto filter_force_command_haptic = new ButterworthFilter(3,0.05);

	double kp_force = 0.0;
	double ki_force = 0.0;
	Vector3d integrated_force_error = Vector3d::Zero();

	// haptic task
	////Haptic teleoperation controller ////
	auto teleop_task = new Sai2Primitives::HapticController(x_init, Matrix3d::Zero());
	teleop_task->_send_haptic_feedback = false;

	//User switch states
	teleop_task->UseGripperAsSwitch();
	bool gripper_state = false;
	bool gripper_state_prev = false;

	 //Task scaling factors
	double Ks = 2.5;
	double KsR = 1.0;
	teleop_task->setScalingFactors(Ks, KsR);

	double kv_haptic = 0.0;

	int contact_transition_counter = 50;

	// // Center of the haptic device workspace
	// Vector3d HomePos_op;
	// HomePos_op << 0.0, 0.0, 0.0;
	// Matrix3d HomeRot_op;
	// HomeRot_op.setIdentity();
	// teleop_task->setDeviceCenter(HomePos_op, HomeRot_op);

	// double force_guidance_position_impedance = 1000.0;
	// double force_guidance_orientation_impedance = 50.0;
	// double force_guidance_position_damping = 5.0;
	// double force_guidance_orientation_damping = 0.1;
	// teleop_task->setVirtualGuidanceGains (force_guidance_position_impedance, force_guidance_position_damping,
	// 								force_guidance_orientation_impedance, force_guidance_orientation_damping);

	VectorXd _max_stiffness_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0]);
	VectorXd _max_damping_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0]);
	VectorXd _max_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0]);

	//set the device specifications to the haptic controller
	teleop_task->_max_linear_stiffness_device = _max_stiffness_device0[0];
	teleop_task->_max_angular_stiffness_device = _max_stiffness_device0[1];
	teleop_task->_max_linear_damping_device = _max_damping_device0[0];
	teleop_task->_max_angular_damping_device = _max_damping_device0[1];
	teleop_task->_max_force_device = _max_force_device0[0];
	teleop_task->_max_torque_device = _max_force_device0[1];

	// setup redis keys to be updated with the callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// Objects to read from redis
    redis_client.addEigenToReadCallback(0, DEVICE_POSITION_KEYS[0], teleop_task->_current_position_device);
    redis_client.addEigenToReadCallback(0, DEVICE_ROTATION_KEYS[0], teleop_task->_current_rotation_device);
    redis_client.addEigenToReadCallback(0, DEVICE_TRANS_VELOCITY_KEYS[0], teleop_task->_current_trans_velocity_device);
    redis_client.addEigenToReadCallback(0, DEVICE_ROT_VELOCITY_KEYS[0], teleop_task->_current_rot_velocity_device);
    redis_client.addEigenToReadCallback(0, DEVICE_SENSED_FORCE_KEYS[0], teleop_task->_sensed_force_device);
    redis_client.addEigenToReadCallback(0, DEVICE_SENSED_TORQUE_KEYS[0], teleop_task->_sensed_torque_device);
    redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_POSITION_KEYS[0], teleop_task->_current_position_gripper_device);
    redis_client.addDoubleToReadCallback(0, DEVICE_GRIPPER_VELOCITY_KEYS[0], teleop_task->_current_gripper_velocity_device);

	// Objects to write to redis
	//write haptic commands
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_FORCE_KEYS[0], teleop_task->_commanded_force_device);
	redis_client.addEigenToWriteCallback(0, DEVICE_COMMANDED_TORQUE_KEYS[0], teleop_task->_commanded_torque_device);
	redis_client.addDoubleToWriteCallback(0, DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], teleop_task->_commanded_gripper_force_device);

	// logger
	string folder = "../../13-LocallySeparatedHapticControl/data_logging/data/";
	string timestamp = currentDateTime();
	string prefix = "data";
	string suffix = ".csv";
	string filename = folder + prefix + "_" + timestamp + suffix;
	auto logger = new Logging::Logger(1000, filename);
	
	logger->addVectorToLog(&log_robot_position, "robot_position");
	logger->addVectorToLog(&log_haptic_position, "haptic_position");
	logger->addVectorToLog(&log_robot_force, "robot_force");
	logger->addVectorToLog(&log_haptic_force, "haptic_force");
	logger->addVectorToLog(&log_sensed_force, "sensed_force");
	logger->addVectorToLog(&log_eigenvalues, "eigenvalues");
	logger->addVectorToLog(&log_eigenvector_0, "eigenvector_0");
	logger->addVectorToLog(&log_eigenvector_1, "eigenvector_1");
	logger->addVectorToLog(&log_eigenvector_2, "eigenvector_2");

	logger->addVectorToLog(&log_force_axis, "force_axis");
	logger->addVectorToLog(&log_motion_axis, "motion_axis");
	logger->addVectorToLog(&log_contact_0, "contact_0");
	logger->addVectorToLog(&log_contact_1, "contact_1");
	logger->addVectorToLog(&log_contact_2, "contact_2");

	logger->addVectorToLog(&log_commfreq_delay_forcespacedim, "comm_freq-delay_ms-force_space_dimension");

	logger->start();

	// create a timer
	double control_loop_freq = 1000.0;
	unsigned long long controller_counter = 0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(control_loop_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while (fSimulationRunning)
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;

		// read haptic state and robot state
		redis_client.executeReadCallback(0);
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		N_prec.setIdentity(dof,dof);
		pos_task->updateTaskModel(N_prec);

		teleop_task->UseGripperAsSwitch();
		gripper_state_prev = gripper_state;
		gripper_state = teleop_task->gripper_state;


		if(state == INIT)
		{

			pos_task->computeTorques(pos_task_torques);
			command_torques = pos_task_torques;

  			// compute homing haptic device
  			teleop_task->HomingTask();

			// if((pos_task->_desired_position - pos_task->_current_position).norm() < 0.2)
			if(teleop_task->device_homed && gripper_state && (pos_task->_desired_position - pos_task->_current_position).norm() < 0.2)
			{
				// Reinitialize controllers
				pos_task->reInitializeTask();
				teleop_task->reInitializeTask();

				pos_task->_kp = 200.0;
				pos_task->_kv = 30.0;

				state = CONTROL;
			}
		}

		else if(state == CONTROL)
		{
			// MatrixXd next_sigma_force = Matrix3d::Zero();
			// if(force_space_dimension == 1)
			// {
			// 	next_sigma_force = force_axis*force_axis.transpose();
			// }
			// else if(force_space_dimension == 2)
			// {
			// 	next_sigma_force = Matrix3d::Identity() - motion_axis*motion_axis.transpose();
			// }
			// else if(force_space_dimension == 3)
			// {
			// 	next_sigma_force = Matrix3d::Identity();
			// }

			// if( (next_sigma_force - sigma_force_global).norm() > 0.1 )
			// {
			// 	cout << "big increment" << endl;
			// 	Matrix3d sigma_force_increment = 0.1 * (next_sigma_force - sigma_force_global)/(next_sigma_force - sigma_force_global).norm();
			// 	sigma_force_global += sigma_force_increment;
			// }
			// else
			// {
			// 	sigma_force_global = next_sigma_force;
			// }
			// sigma_motion_global = Matrix3d::Identity() - sigma_force_global;

			// pos_task->_sigma_force = sigma_force_global;
			// pos_task->_sigma_motion = sigma_motion_global;





			if(force_space_dimension == 1)
			{
				pos_task->setForceAxis(force_axis);
			}
			else if(force_space_dimension == 2)
			{
				pos_task->setMotionAxis(motion_axis);
			}
			else if(force_space_dimension == 3)
			{
				pos_task->setFullForceControl();
			}
			else
			{
				pos_task->setFullMotionControl();
			}
			sigma_force_global = pos_task->_sigma_force;


			Vector3d desired_position = Vector3d::Zero();
			teleop_task->computeHapticCommands3d(desired_position);
			haptic_position_global = desired_position;

			haptic_velocity_global = teleop_task->_current_trans_velocity_device_RobFrame;
			// Vector3d desired_velocity = teleop_task->_current_trans_velocity_device_RobFrame;
			// desired_position = teleop_task->_current_position_device;

			pos_task->_desired_position = delayed_haptic_position;
			pos_task->_desired_velocity = delayed_haptic_velocity;

			robot_pos_error = pos_task->_current_position - pos_task->_desired_position;

			Vector3d force_command_robot = pos_task->_sigma_force * k_vir_robot * robot_pos_error;
			Vector3d force_diff_robot = force_command_robot - prev_force_command_robot;
			// if( force_diff_robot.norm() > max_force_diff_robot )
			// {
			// 	// cout << "prev force :\n" << prev_force_command_robot.transpose() << endl;
			// 	// cout << "new force command :\n" << force_command.transpose() << endl;
			// 	// cout << endl;
			// 	force_command_robot = prev_force_command_robot + max_force_diff_robot * force_diff_robot/force_diff_robot.norm();
			// }
			if(force_command_robot.norm() > 10.0)
			{
				force_command_robot *= 10.0/force_command_robot.norm();
			}

			Vector3d force_error = pos_task->_sigma_force * (sensed_force_moment.head(3) - force_command_robot);
			integrated_force_error += force_error * 0.001;

			force_command_robot -= pos_task->_sigma_force * (kp_force * force_error + ki_force * integrated_force_error);
			pos_task->_desired_force = filter_force_command_robot->update(-force_command_robot);

			pos_task->computeTorques(pos_task_torques);
			command_torques = pos_task_torques;
			robot_position_global = pos_task->_current_position;

			haptic_pos_error = delayed_robot_position - desired_position;
			Vector3d force_command_haptic = delayed_sigma_force * k_vir_robot * haptic_pos_error;
			Vector3d force_diff_haptic = force_command_haptic - prev_force_command_haptic;
			// if( force_diff_haptic.norm() > max_force_diff_haptic )
			// {
			// 	// cout << "prev force :\n" << prev_force_command_robot.transpose() << endl;
			// 	// cout << "new force command :\n" << force_command.transpose() << endl;
			// 	// cout << endl;
			// 	force_command_haptic = prev_force_command_robot + max_force_diff_haptic * force_diff_haptic/force_diff_haptic.norm();
			// }
			if(force_command_haptic.norm() > 10.0)
			{
				force_command_haptic *= 10.0/force_command_haptic.norm();
			}

			// teleop_task->_commanded_force_device = filter_force_command_haptic->update(force_command_haptic) - delayed_sigma_force * kv_haptic * teleop_task->_current_trans_velocity_device;
			teleop_task->_commanded_force_device = filter_force_command_haptic->update(force_command_haptic);
			// teleop_task->_commanded_force_device = -sensed_force_moment.head(3)/Ks - 5.0 * teleop_task->_current_trans_velocity_device;
			// teleop_task->_commanded_force_device = -delayed_sensed_force/Ks;





			haptic_PO += teleop_task->_commanded_force_device.dot(teleop_task->_current_trans_velocity_device) * 0.001;

			double vh_square = (delayed_sigma_force * teleop_task->_current_trans_velocity_device).squaredNorm();
			double alpha_pc = 0;
			if(haptic_PO < 0 && vh_square > 0.0001)
			{
				alpha_pc = -haptic_PO / vh_square * 1000.0;
			}
			if(alpha_pc > 0.9 * _max_damping_device0[0] - kv_haptic)
			{
				alpha_pc = 0.9 * _max_damping_device0[0] - kv_haptic;
			}
			if(alpha_pc < 0)
			{
				alpha_pc = 0;
			}

			teleop_task->_commanded_force_device -= alpha_pc * delayed_sigma_force * teleop_task->_current_trans_velocity_device;
			haptic_PO -= alpha_pc * vh_square * 0.001;









			// double haptic_power_input = teleop_task->_current_trans_velocity_device.dot(teleop_task->_commanded_force_device);
			// double haptic_PO_prev = haptic_PO;
			// haptic_PO += haptic_power_input;

			// // if(haptic_PO > 0)
			// // {
			// // 	kv_haptic += 1.0;
			// // }
			// // else
			// // {
			// // 	kv_haptic -= 0.1;
			// // }

			// double xh_dor_square = (delayed_sigma_force*teleop_task->_current_trans_velocity_device).squaredNorm();
			// haptic_PO -= kv_haptic * xh_dor_square / control_loop_freq;
			
			// // if(teleop_task->_commanded_force_device.dot(force_axis) > 0)
			// // if(delayed_sensed_force.norm() > 0)
			// {
			// 	teleop_task->_commanded_force_device -= kv_haptic * delayed_sigma_force * teleop_task->_current_trans_velocity_device;
			// // 	Vector3d damping = delayed_sensed_force.dot(teleop_task->_current_trans_velocity_device)/delayed_sensed_force.norm() * teleop_task->_current_trans_velocity_device;
			// // 	teleop_task->_commanded_force_device -= 15.0 * damping;
			// }

			// if(force_space_dimension != previous_force_space_dimension && force_space_dimension != 0)
			if(force_space_dimension > previous_force_space_dimension)
			{
				contact_transition_counter = 50;
			}
			else
			{
				contact_transition_counter--;
			}

			if(contact_transition_counter > 0)
			{
				teleop_task->_commanded_force_device = -15.0 * delayed_sigma_force * teleop_task->_current_trans_velocity_device;
			}
			else
			{
				contact_transition_counter = 0;
			}




			// if(haptic_PO == haptic_PO_prev)
			// {
			// 	haptic_PO = 0;
			// }

			// if(haptic_power_input > 0)
			// {
			// 	kv_haptic += 1;
			// }
			// else
			// {
			// 	kv_haptic -= 0.5;
			// }
			// if(kv_haptic > 20.0)
			// {
			// 	kv_haptic = 20.0;
			// }
			// if(kv_haptic < 0.0)
			// {
			// 	kv_haptic = 0.0;
			// }

			// cout << "PO haptic : " << haptic_PO << endl;
			// cout << "kv_haptic : " << kv_haptic << endl;
			// cout << endl;

			// cout << "k virtual :\n" << k_vir_haptic << endl;

			// previous_force_space_dimension = force_space_dimension;
			// adding_contact = false;
			// removing_contact = false;
			// 
			prev_force_command_robot = force_command_robot;
			prev_force_command_haptic = force_command_haptic;

		}

		// particle filter
		motion_control_pfilter = pos_task->_sigma_motion * pos_task->_motion_control;
		// motion_control_pfilter += pos_task->_sigma_motion * pos_task->_motion_control / control_loop_freq;
		// motion_control_pfilter = pos_task->_sigma_motion * motion_control_pfilter;
		force_control_pfilter = pos_task->_sigma_force * pos_task->_force_control;
		measured_velocity_pfilter = pos_task->_current_velocity;
		measured_force_pfilter = sensed_force_moment.head(3);

		// if( teleop_task->_current_position_device(2) < -0.0315)
		// {
		// 	cout << "motion control :\n" << motion_control_pfilter.transpose() << endl;
		// 	cout << "force control robot :\n" << force_control_pfilter.transpose() << endl;
		// 	cout << "force control haptic :\n" << teleop_task->_commanded_force_device.transpose() << endl;
		// 	cout << "position haptic :\n" << teleop_task->_current_position_device.transpose() << endl;
		// 	cout << "sigma force :\n" << pos_task->_sigma_force << endl;
		// 	cout << "sigma motion :\n" << pos_task->_sigma_motion << endl;
		// 	// cout << "lambda motion control :\n" << (pos_task->_Lambda * motion_control_pfilter).transpose() << endl;
		// 	cout << endl;
		// }

		// write control torques
		redis_client.executeWriteCallback(0);
		sim->setJointTorques(robot_name, command_torques + fsensor_torques);

		// logger
		log_robot_position = pos_task->_current_position;
		log_haptic_position = haptic_position_global;
		log_robot_force = pos_task->_desired_force;
		log_haptic_force = teleop_task->_commanded_force_device;
		log_sensed_force = -sensed_force_moment.head(3);

		// // cout statements
		// if(controller_counter % 500 == 0)
		// {
		// 	cout << "controller counter : " << controller_counter << endl;
		// 	cout << "desired force : " << pos_task->_desired_force.transpose() << endl;
		// 	cout << endl;
		// }

		controller_counter++;

	}

	logger->stop();

	//// Send zero force/torque to robot and haptic device through Redis keys ////
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0], Vector3d::Zero());
	redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0], Vector3d::Zero());
	redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], "0.0");

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


}

void communication()
{
	const double communication_delay_ms = 0;

	queue<Vector3d> robot_position_buffer;
	queue<Vector3d> haptic_position_buffer;
	queue<Vector3d> haptic_velocity_buffer;
	queue<Vector3d> sensed_force_buffer;
	queue<Matrix3d> sigma_force_buffer;

	// create a timer
	double communication_freq = 50.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(communication_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	log_commfreq_delay_forcespacedim(0) = communication_freq;
	log_commfreq_delay_forcespacedim(1) = communication_delay_ms;

	unsigned long long communication_counter = 0;
	const int communication_delay_ncycles = communication_delay_ms / 1000.0 * communication_freq;
	
	while(fSimulationRunning)
	{
		timer.waitForNextLoop();

		if(communication_delay_ncycles == 0)
		{
			delayed_haptic_position = haptic_position_global;
			delayed_haptic_velocity = haptic_velocity_global;
			delayed_robot_position = robot_position_global;
			delayed_sensed_force = sensed_force_global;
			delayed_sigma_force = sigma_force_global;
		}
		else
		{
			haptic_position_buffer.push(haptic_position_global);
			haptic_velocity_buffer.push(haptic_velocity_global);
			robot_position_buffer.push(robot_position_global);
			sensed_force_buffer.push(sensed_force_global);
			sigma_force_buffer.push(sigma_force_global);

			if(communication_counter > communication_delay_ncycles)
			{
				delayed_haptic_position = haptic_position_buffer.front();
				delayed_haptic_velocity = haptic_velocity_buffer.front();
				delayed_robot_position = robot_position_buffer.front();
				delayed_sensed_force = sensed_force_buffer.front();
				delayed_sigma_force = sigma_force_buffer.front();

				haptic_position_buffer.pop();
				haptic_velocity_buffer.pop();
				robot_position_buffer.pop();
				sensed_force_buffer.pop();
				sigma_force_buffer.pop();

				communication_counter--;
			}

		}


		communication_counter++;
	}


}

void particle_filter()
{
	unsigned long long pf_counter = 0;

	vector<Vector3d> cluster_centers;
	const double interpolation_step = 0.1; 

	Matrix3d sigma_force_goal = Matrix3d::Zero();
	Matrix3d sigma_force_prev_goal = Matrix3d::Zero();

	auto pfilter = new ForceSpaceParticleFilter_weight_mem(n_particles);


	// create a timer
	double pfilter_freq = 100.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(pfilter_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while(fSimulationRunning)
	{
		timer.waitForNextLoop();
		// vector<pair<Vector3d, double>> weighted_particles = particleFilterMotionUpdate(particles, motion_control_pfilter, force_control_pfilter, measured_velocity_pfilter, measured_force_pfilter, force_space_dimension);
		// // vector<Vector3d> new_particles = particleFilterResampling(weighted_particles, n_particles);
		// vector<pair<Vector3d,double>> new_particles_with_weights = particleFilterResamplingLowVariance(weighted_particles, n_particles);

		// // PCA
		// MatrixXd points_to_PCA = MatrixXd::Zero(3, 1.5*n_particles);
		// for(int i=0 ; i<n_particles ; i++)
		// {
		// 	points_to_PCA.col(i) = new_particles_with_weights[i].first;
		// }
		// MatrixXd centered_points_to_PCA = points_to_PCA.colwise() - points_to_PCA.rowwise().mean();

		// Matrix3d cov = centered_points_to_PCA * centered_points_to_PCA.transpose();

		// SelfAdjointEigenSolver<MatrixXd> eig(cov);

		// // Get the eigenvectors and eigenvalues.
		// MatrixXd evecs = eig.eigenvectors();
		// VectorXd evals = eig.eigenvalues();
		// if(evals.sum() > 1)
		// {
		// 	evals /= evals.sum();
		// }


		pfilter->_force_space_dimension = force_space_dimension;
		pfilter->_force_axis = force_axis;
		pfilter->_motion_axis = motion_axis;

		pfilter->update(motion_control_pfilter, force_control_pfilter, measured_velocity_pfilter, measured_force_pfilter);

		Matrix3d evecs = Matrix3d::Zero();
		Vector3d evals = Vector3d::Zero();
		pfilter->computePCA(evals, evecs);

		double force_space_dimension_up = 0;
		double force_space_dimension_down = 0;
		for(int i=0 ; i<3 ; i++)
		{
			if(evals(i) > 0.25)
			{
				force_space_dimension_up++;
			}
			if(evals(i) > 0.05)
			{
				force_space_dimension_down++;
			}
		}

		if(force_space_dimension_up == force_space_dimension_down)
		{
			force_space_dimension = force_space_dimension_down;
		}
		else
		{
			if(force_space_dimension >= force_space_dimension_down)
			{
				force_space_dimension = force_space_dimension_down;
			}
			else if(force_space_dimension <= force_space_dimension_up)
			{
				force_space_dimension = force_space_dimension_up;
			}
		}

		Vector3d force_axis_tmp = evecs.col(2);
		Vector3d motion_axis_tmp = evecs.col(0);


		if(force_axis_tmp(2) < -0.01)
		{
			force_axis = -force_axis_tmp;
		}
		else if(force_axis_tmp(1) < -0.01)
		{
			force_axis = -force_axis_tmp;
		}
		else if(force_axis_tmp(0) < -0.01)
		{
			force_axis = -force_axis_tmp;
		}
		else
		{
			force_axis = force_axis_tmp;
		}

		if(motion_axis_tmp(1) < -0.01)
		{
			motion_axis = -motion_axis_tmp;
		}
		else if(motion_axis_tmp(2) < -0.01)
		{
			motion_axis = -motion_axis_tmp;
		}
		else if(motion_axis_tmp(0) < -0.01)
		{
			motion_axis = -motion_axis_tmp;
		}
		else
		{
			motion_axis = motion_axis_tmp;
		}



		// final resampling
		vector<Vector3d> new_particles;
		// if(force_space_dimension < 1)
		{
			for(int i=0 ; i<n_particles ; i++)
			{
				// new_particles.push_back(new_particles_with_weights[i].first);
				new_particles.push_back(pfilter->_particles[i]);
			}
		}


		Vector3d robot_pos_in_world = robot_position_global + Vector3d(0, 0, 0.15);
		Vector3d tentative_particle = Vector3d(0, 0, -0.5) - robot_pos_in_world;
		tentative_particle.normalize();

		double weight_force_tentative_particle = 1.2 * tentative_particle.dot(measured_force_pfilter.normalized());
		double weight_velocity_tetnative_particle = 1 - abs(measured_velocity_pfilter.normalized().dot(tentative_particle));
		// double weight_velocity_tetnative_particle = 1 - abs(tanh(15.0*measured_velocity_pfilter.dot(tentative_particle)));



		// if(pf_counter % 10 == 0)
		// if(force_space_dimension != 1)
		// {
		// 	cout << "********************************" << endl;
		// 	cout << "pf coutner : " << pf_counter << endl;
		// 	cout << "contact space dim : " << force_space_dimension << endl;
		// 	cout << "previous contact space dim : " << previous_force_space_dimension << endl;
		// 	cout << "force spce dimension up : " << force_space_dimension_up << endl;
		// 	cout << "force spce dimension down : " << force_space_dimension_down << endl;
		// 	cout << "eigenvalues :\n" << evals.transpose() << endl;
		// 	cout << "eigenvectors :\n" << evecs << endl;
		// 	cout << "tentative_particle :\n" << tentative_particle.transpose() << endl;
		// 	cout << "weight force : " << weight_force_tentative_particle << endl;
		// 	cout << "weight velocity : " << weight_velocity_tetnative_particle << endl;
		// 	// cout << "cluster center size : " << cluster_centers.size() << endl;
		// 	// cout << "n particles per cluster : " << num_particle_in_cluster.transpose() << endl;
		// 	// cout << "eigenvectors rotation test :\n" << evecs*evecs.transpose() << endl;
		// 	// cout << "force_axis :\n" << force_axis.transpose() << endl;
		// 	// cout << "motion_axis :\n" << motion_axis.transpose() << endl;
		// 	cout << endl;
		// }

		// logger
		log_eigenvalues = evals;
		log_eigenvector_0 = evecs.col(0);
		log_eigenvector_1 = evecs.col(1);
		log_eigenvector_2 = evecs.col(2);
		log_commfreq_delay_forcespacedim(2) = force_space_dimension;

		log_force_axis = force_axis;
		log_motion_axis = motion_axis;

		if(previous_force_space_dimension == 1 && force_space_dimension == 0)
		{
			cout << "********************************" << endl;
			cout << "pf coutner : " << pf_counter << endl;
			cout << "contact space dim : " << force_space_dimension << endl;
			cout << "previous contact space dim : " << previous_force_space_dimension << endl;
			cout << "force spce dimension up : " << force_space_dimension_up << endl;
			cout << "force spce dimension down : " << force_space_dimension_down << endl;
			cout << "eigenvalues :\n" << evals.transpose() << endl;
			cout << "eigenvectors :\n" << evecs << endl;
			// fSimulationRunning = false;
		}

		previous_force_space_dimension = force_space_dimension;
		particles = new_particles;

		pf_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Particle Filter Loop run time  : " << end_time << " seconds\n";
	std::cout << "Particle Filter Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Particle Filter Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor) {

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);

	// sensed force
	Vector3d sensed_force = Vector3d::Zero();
	Vector3d sensed_moment = Vector3d::Zero();

	// contact list
	vector<Vector3d> contact_points;
	vector<Vector3d> contact_forces;

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

		// sim->setJointTorques(robot_name, command_torques);

		// integrate forward
		// double curr_time = timer.elapsedTime();
		// double loop_dt = curr_time - last_time;
		// sim->integrate(loop_dt);
		sim->integrate(1.0/sim_frequency);

		// get contacts for logging
		contact_points.clear();
		contact_forces.clear();
		sim->getContactList(contact_points, contact_forces, robot_name, link_name);

		log_commfreq_delay_forcespacedim(3) = contact_points.size();

		log_contact_0.setZero();
		log_contact_1.setZero();
		log_contact_2.setZero();
		Vector3d robot_pos_in_world = robot_position_global + Vector3d(0, 0, 0.15);

		// Vector3d p_sphere_cylinder = Vector3d(0.0, 0.0, -0.5) - robot_pos_in_world;
		// p_sphere_cylinder(0) = 0;
		// log_contact_0 = p_sphere_cylinder.normalized();
		// log_contact_0(0) = 0;
		// log_contact_0.normalize();

		if(contact_points.size() > 0)
		{
			log_contact_0 = contact_points[0] - robot_pos_in_world;
			log_contact_0.normalize();
			// cout << "contact point :\n" << contact_points[0].transpose() << endl;
			// cout << "robot position :\n" << robot_pos_in_world.transpose() << endl; 
			// cout << "contact0 :\n" << log_contact_0.transpose() << endl;
		}
		if(contact_points.size() > 1)
		{
			log_contact_1 = contact_points[1] - robot_pos_in_world;
			log_contact_1.normalize();
		}
		if(contact_points.size() > 2)
		{
			log_contact_2 = contact_points[2] - robot_pos_in_world;
			log_contact_2.normalize();
		}

		// sim->showContactInfo();

		// read joint positions, velocities, update model
		// sim->getJointPositions(robot_name, robot->_q);
		// sim->getJointVelocities(robot_name, robot->_dq);
		// robot->updateKinematics();

		// read end-effector task forces from the force sensor simulation
		force_sensor->update(sim);
		force_sensor->getForceLocalFrame(sensed_force);
		force_sensor->getMomentLocalFrame(sensed_moment);
		
		// sensed_force.setZero();
		// cout << p_sphere_cylinder.transpose() << endl;
		// cout << p_sphere_cylinder.norm() << endl;
		// if(p_sphere_cylinder.norm() < 0.65)
		// {
		// 	sensed_force -= 1.0 * (0.65 - p_sphere_cylinder.norm()) * p_sphere_cylinder.normalized();
		// }

		// MatrixXd Jv = MatrixXd::Zero(3,3);
		// robot->JvWorldFrame(Jv, link_name);

		// fsensor_torques = Jv.transpose() * sensed_force;

		sensed_force_moment << -sensed_force, -sensed_moment;
		sensed_force_global = sensed_force_moment.head(3);


		// cout << sensed_force_moment.transpose() << endl;

		//update last time
		// last_time = curr_time;

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
