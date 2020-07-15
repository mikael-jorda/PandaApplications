#include "MomentumObserver.h"
#include <stdexcept>

MomentumObserver::MomentumObserver(Sai2Model::Sai2Model* robot, double update_rate)
{
	_robot = robot;
	int dof = _robot->dof();

	_K0 = MatrixXd::Identity(dof,dof);

	_dt = update_rate;

	reInitialize();
}

MomentumObserver::~MomentumObserver(){}

void MomentumObserver::reInitialize()
{
	int dof = _robot->dof();
	_beta.setZero(dof);
	_rho.setZero(dof);
	_integrated_rho_hat.setZero(dof);
	_r.setZero(dof);

	_rho_0 = _robot->_M * _robot->_dq;
}


void MomentumObserver::setGain(MatrixXd K)
{
	int dof = _robot->dof();
	if(K.rows() != dof || K.cols() != dof)
	{
		throw std::invalid_argument("size of gain matrix inconsistent with robot model in MomentumObserver::setGain()\n");
	}

	_K0 = K;
}

void MomentumObserver::update(VectorXd& command_torques)
{
	VectorXd tau_contact = VectorXd::Zero(_robot->dof());
	update(command_torques, tau_contact);
}

void MomentumObserver::update(VectorXd& command_torques, VectorXd& known_contact_torques)
{
	int dof = _robot->dof();

	_rho = _robot->_M * _robot->_dq;

	MatrixXd C = MatrixXd::Zero(dof,dof);
	_robot->factorizedChristoffelMatrix(C);
	VectorXd g = VectorXd::Zero(dof);
	_robot->gravityVector(g);
	_beta = g - C.transpose()*_robot->_dq;

	VectorXd rho_hat_dot = command_torques - _beta - known_contact_torques + _r;
	_integrated_rho_hat += rho_hat_dot * _dt;

	_r = _K0 * (_rho - _rho_0 - _integrated_rho_hat);
}

VectorXd MomentumObserver::getDisturbanceTorqueEstimate()
{
	return -_r;
}