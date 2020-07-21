#include "ForceSpaceParticleFilter_weight_mem.h"

ForceSpaceParticleFilter_weight_mem::ForceSpaceParticleFilter_weight_mem(const int n_particles)
{
	_n_particles = n_particles;
	for(int i=0 ; i<_n_particles ; i++)
	{
		_particles.push_back(Vector3d::Zero());
	}

	_mean_scatter = 0.0;
	_std_scatter = 0.005;

	_coeff_friction = 0.0;
}

void ForceSpaceParticleFilter_weight_mem::update(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured)
{

	resamplingLowVariance(motionUpdateAndWeighting(motion_control, force_control, velocity_measured, force_measured));

}

vector<pair<Vector3d, double>> ForceSpaceParticleFilter_weight_mem::motionUpdateAndWeighting(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured)
{
	Vector3d motion_control_normalized = Vector3d::Zero();
	Vector3d force_control_normalized = Vector3d::Zero();
	Vector3d measured_velocity_normalized = Vector3d::Zero();
	Vector3d measured_force_normalized = Vector3d::Zero();

	if(motion_control.norm() > 0.5)
	{
		motion_control_normalized = motion_control/motion_control.norm();
	}
	if(force_control.norm() > 0.5)
	{
		force_control_normalized = force_control/force_control.norm();
	}
	if(velocity_measured.norm() > 1e-3)
	{
		measured_velocity_normalized = velocity_measured/velocity_measured.norm();
	}
	if(force_measured.norm() > 0.5)
	{
		measured_force_normalized = force_measured/force_measured.norm();
	}

	vector<Vector3d> augmented_particles = _particles;

	// add a particle at the center in case of contact loss
	augmented_particles.push_back(Vector3d::Zero());

	// add particles in the direction of the motion control if there is no velocity in that direction
	double prob_add_particle = (1 - abs(tanh(5.0*velocity_measured.dot(motion_control_normalized)))) * (tanh(motion_control_normalized.dot(force_measured)));
	// double prob_add_particle = (1 - abs(tanh(velocity_measured.dot(motion_control_normalized))));
	if(prob_add_particle < 0)
	{
		prob_add_particle = 0;
	}
	int n_added_particles = prob_add_particle * _n_particles;
	for(int i=0 ; i<n_added_particles ; i++)
	{
		double alpha = (double) (i + 0.5) / (double)n_added_particles; // add particles on the arc betwen the motion and force control
		Vector3d new_particle = (1 - alpha) * motion_control_normalized + alpha * force_control_normalized;
		new_particle.normalize();
		augmented_particles.push_back(new_particle);
	}

	int n_new_particles = 1 + n_added_particles;

	// prepare weights
	vector<pair<Vector3d, double>> weighted_particles;
	double cumulative_weight = 0;

	for(int i=0 ; i< _n_particles + n_new_particles ; i++)
	{
		// control update : scatter the particles that are not at the center
		Vector3d current_particle = augmented_particles[i];

		if(current_particle.norm() > 1e-3) // contact
		{
			double normal_rand_1 = sampleNormalDistribution(_mean_scatter, _std_scatter);
			double normal_rand_2 = sampleNormalDistribution(_mean_scatter, _std_scatter);
			double normal_rand_3 = sampleNormalDistribution(_mean_scatter, _std_scatter);
			current_particle += Vector3d(normal_rand_1, normal_rand_2, normal_rand_3);

			current_particle.normalize();
		}

		// measurement update : compute weight due to force measurement
		double weight_force = 0;
		if(current_particle.norm() < 1e-3)
		{
			weight_force = 1 - tanh(force_measured.norm());
		}
		else
		{
			weight_force = 1.3 * tanh(current_particle.dot(force_measured));
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
		double weight_velocity = 1 - abs(tanh(15.0*velocity_measured.dot(current_particle)));

		// final weight
		double weight = weight_force * weight_velocity;
		cumulative_weight += weight;

		weighted_particles.push_back(make_pair(current_particle, cumulative_weight));
	}

	for(int i=0 ; i< _n_particles + n_new_particles ; i++)
	{
		weighted_particles[i].second /= cumulative_weight;
	}

	return weighted_particles;
}

void ForceSpaceParticleFilter_weight_mem::resamplingLowVariance(vector<pair<Vector3d, double>> weighted_particles)
{
	int n_weighted_particles = weighted_particles.size();

	double n_inv = 1.0/(double)_n_particles;
	double r = sampleUniformDistribution(0,n_inv);
	int k = 0;

	for(int i=0 ; i<_n_particles ; i++)
	{
		while(r > weighted_particles[k].second)
		{
			k++;
		}
		_particles[i] = weighted_particles[k].first;
		r += n_inv;
	}
}

void ForceSpaceParticleFilter_weight_mem::computePCA(Vector3d& eigenvalues, Matrix3d& eigenvectors)
{
	MatrixXd points_to_PCA = MatrixXd::Zero(3, 1.5*_n_particles);
	for(int i=0 ; i<_n_particles ; i++)
	{
		points_to_PCA.col(i) = _particles[i];
	}

	MatrixXd centered_points_to_PCA = points_to_PCA.colwise() - points_to_PCA.rowwise().mean();
	Matrix3d cov = centered_points_to_PCA * centered_points_to_PCA.transpose();

	SelfAdjointEigenSolver<MatrixXd> eig(cov);

	// Get the eigenvectors and eigenvalues.
	eigenvectors = eig.eigenvectors();
	eigenvalues = eig.eigenvalues();
}
