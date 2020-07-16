#include <iostream>
#include <stdexcept>

#include "KalmanFilter.h"

namespace KalmanFilters
{

// KalmanFilter::KalmanFilter() {}

KalmanFilter::KalmanFilter(
	const double dt,
	const Eigen::MatrixXd& F,
	const Eigen::MatrixXd& H,
	const Eigen::MatrixXd& Q,
	const Eigen::MatrixXd& R)
: _F(F), _H(H), _Q(Q), _R(R),
  _m(H.rows()), _n(F.rows()), _dt(dt), _initialized(false),
  _x_hat(_n), _x_hat_new(_n), _P(_n,_n)

{
 	_I.setIdentity(_n,_n);
}


void KalmanFilter::init() 
{
 	_x_hat.setZero();
 	_P.setZero(_n,_n);
 	_t = 0;
 	_initialized = true;
}

void KalmanFilter::init(const Eigen::VectorXd& x0, const double t0)
{
 	_x_hat = x0;
 	_P.setZero(_n,_n);
 	_t = t0;
 	_initialized = true;
}

void KalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0, const double t0)
{
 	_x_hat = x0;
 	_P = P0;
 	_t = t0;
 	_initialized = true;
}


void KalmanFilter::update(const Eigen::VectorXd& y)
{

 	if(!_initialized)
 		throw std::runtime_error("Kalman Filter is not initialized!");
 	//First phase: Prediction
 	_x_hat_new = _F * _x_hat;
 	_P = _F*_P*_F.transpose() + _Q;

 	//Second phase: Correction of predicted variables based on Kalman gain
 	Eigen::MatrixXd K = _P*_H.transpose()*(_H*_P*_H.transpose() + _R).inverse(); //Kalman gain
 	_x_hat_new += K * (y - _H*_x_hat_new);
 	_P = (_I - K*_H)*_P;
 	_x_hat = _x_hat_new; 

 	_t += _dt; 
}


Eigen::VectorXd KalmanFilter::getState() 
{
	return _x_hat;
}

double KalmanFilter::getTime()
{
	return _t;
}

} /* namespace KalmanFilters */