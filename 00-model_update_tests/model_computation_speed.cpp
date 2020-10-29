#include "Sai2Model.h"
// #include "redis/RedisClient.h"
// #include "timer/LoopTimer.h"
// #include "tasks/JointTask.h"

#include <iostream>
#include <string>
#include <chrono>

// #include <signal.h>
// bool runloop = true;
// void sighandler(int sig)
// { runloop = false; }

using namespace std;
using namespace Eigen;

const string panda_file = "./resources/panda_arm.urdf";

vector<string> robot_names ={
	"panda_arm",
	"two_pandas_mobile",
	"humanoid",
	"two_pandas_mobile_hands",
	"humanoid_hands",
};

const string link_name = "link7_arm1";

int main() {

	const int n_iterations = 1000000;

	vector<Sai2Model::Sai2Model*> robots;

	// load robots
	for(int i=0 ; i<robot_names.size() ; i++)
	{
		string filename = "./resources/" + robot_names[i] + ".urdf";
		robots.push_back(new Sai2Model::Sai2Model(filename, false));
	}


	auto t_start = std::chrono::high_resolution_clock::now();
    auto t_end = std::chrono::high_resolution_clock::now();

	for(int i=0 ; i<robot_names.size() ; i++)
	{
		vector<int> computation_times_model;
		vector<int> computation_times_task;
		double mean_duration_model = 0;
		double mean_duration_task = 0;

		MatrixXd jacobian = MatrixXd::Zero(6,robots[i]->dof());
		MatrixXd Lambda = MatrixXd::Zero(6,6);
		MatrixXd J_bar = MatrixXd::Zero(robots[i]->dof(),6);
		MatrixXd N = MatrixXd::Zero(robots[i]->dof(),robots[i]->dof());

		for(int j=0 ; j<n_iterations ; j++)
		{
			robots[i]->_q = M_PI / 2 * VectorXd::Random(robots[i]->dof());

			t_start = std::chrono::high_resolution_clock::now();
			robots[i]->updateModel();
			t_end = std::chrono::high_resolution_clock::now();

		    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t_end - t_start ).count();

		    computation_times_model.push_back(duration);
		    mean_duration_model += duration;


			robots[i]->J_0(jacobian, link_name);
			t_start = std::chrono::high_resolution_clock::now();
			robots[i]->operationalSpaceMatrices(Lambda, J_bar, N, jacobian);
			t_end = std::chrono::high_resolution_clock::now();

		    duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t_end - t_start ).count();

		    computation_times_task.push_back(duration);
		    mean_duration_task += duration;


		}

		mean_duration_model /= n_iterations;
		// compute std model
		double std_duration_model = 0;
		for(int j=0 ; j<n_iterations ; j++)
		{
			std_duration_model += (computation_times_model[j] - mean_duration_model) * (computation_times_model[j] - mean_duration_model);
		}
		std_duration_model /= n_iterations;
		std_duration_model = sqrt(std_duration_model);


		mean_duration_task /= n_iterations;
		// compute std task
		double std_duration_task = 0;
		for(int j=0 ; j<n_iterations ; j++)
		{
			std_duration_task += (computation_times_task[j] - mean_duration_task) * (computation_times_task[j] - mean_duration_task);
		}
		std_duration_task /= n_iterations;
		std_duration_task = sqrt(std_duration_task);


		cout << "robot : " << robot_names[i] << endl;
		cout << "number of DoF : " << robots[i]->dof() << endl;
		cout << "average model update time (nanoseconds) : " << mean_duration_model << endl;
		cout << "standard deviation model update time (nanoseconds) : " << std_duration_model << endl;
		cout << "average task update time (nanoseconds) : " << mean_duration_task << endl;
		cout << "standard deviation task update time (nanoseconds) : " << std_duration_task << endl;
		cout << endl;


	}

	return 0;
}
