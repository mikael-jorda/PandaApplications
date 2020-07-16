
#ifndef KALMAN_FILTERS_KALMAN_FILTER_H_
#define KALMAN_FILTERS_KALMAN_FILTER_H_


#include <math.h>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>

namespace KalmanFilters
{
class KalmanFilter {
public: 

	/** 
	* Empty default constructor
	*/
	// KalmanFilter();

	/**
	Constructor Kalman filter with the following matrices:
	* 	F - dynamics matrix (A)
	*	H - output matrix (C)
	*	Q - process noise covariance matrix
	*	R - measurement noise covariance matrix
	*/
	KalmanFilter(
		const double dt,
		const Eigen::MatrixXd& F,
		const Eigen::MatrixXd& H,
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R
		);


	/** 
	* Initialize filter with initial states and intial error covariance as zero 
	*/
	void init();

	/** 
	* Initialize filter with intial error covariance as zero 
	*/
	void init(const Eigen::VectorXd& x0, const double t0 = 0);

	/** 
	* Initialize filter with a guess for initial states and error covariance
	*/
	void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0, const double t0 = 0);

	/**
	* Update the estimated state based on measured values
	* time step is assumed to remain constant
	*/
	void update(const Eigen::VectorXd& y);

	/**
	* return current state and time
	*/
	Eigen::VectorXd getState();
	
	double getTime();

private:

	//Matrices for computation
	Eigen::MatrixXd _F, _H, _Q, _R, _P;

	//System dimensions
	int _n, _m;

	//Current time
	double _t;

	//discrete time step
	double _dt;

	//Is the filter initialized?
	bool _initialized;

	//n-size idenity
	Eigen::MatrixXd _I;

	//Estimated states
	Eigen::VectorXd _x_hat, _x_hat_new;


};
} /* namespace KalmanFilters */


#endif //KALMAN_FILTERS_KALMAN_FILTER_H_