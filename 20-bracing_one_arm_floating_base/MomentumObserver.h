#ifndef MOMENTUMOBSERVER_H_
#define MOMENTUMOBSERVER_H_

#include "Sai2Model.h"
#include <Eigen/Dense>
#include <string>

using namespace std;
using namespace Eigen;

class MomentumObserver {
public:

	MomentumObserver(Sai2Model::Sai2Model* robot, double update_rate);

	~MomentumObserver();

	void reInitialize();

	void setGain(MatrixXd K);

	void update(VectorXd& command_torques);
	void update(VectorXd& command_torques, VectorXd& known_contact_torques);

	VectorXd getDisturbanceTorqueEstimate();

	Sai2Model::Sai2Model* _robot;

	MatrixXd _K0;

	VectorXd _beta;
	VectorXd _rho;
	VectorXd _integrated_rho_hat;
	VectorXd _r;
	VectorXd _rho_0;

	double _dt;
};


#endif /* MOMENTUMOBSERVER_H_ */