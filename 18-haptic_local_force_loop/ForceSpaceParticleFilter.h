

#ifndef FORCE_SPACE_PARTICLE_FILTER_H_
#define FORCE_SPACE_PARTICLE_FILTER_H_

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <random>

using namespace Eigen;
using namespace std;

class ForceSpaceParticleFilter
{
public:

	ForceSpaceParticleFilter(const int n_particles);
	~ForceSpaceParticleFilter(){}

	void update(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured);

	vector<pair<Vector3d, double>> motionUpdateAndWeighting(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured);

	void resamplingLowVariance(vector<pair<Vector3d, double>> weighted_particles);

	void computePCA(Vector3d& eigenvalues, Matrix3d& eigenvectors);


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



	int _n_particles;
	vector<Vector3d> _particles;

	double _mean_scatter;
	double _std_scatter;

	double _coeff_friction;

};

/* FORCE_SPACE_PARTICLE_FILTER_H_ */
#endif
