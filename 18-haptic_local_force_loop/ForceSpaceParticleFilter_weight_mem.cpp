#include "ForceSpaceParticleFilter_weight_mem.h"

#include <iostream>

using namespace Eigen;
using namespace std;

ForceSpaceParticleFilter_weight_mem::ForceSpaceParticleFilter_weight_mem(const int n_particles)
{
	_n_particles = n_particles;
	for(int i=0 ; i<_n_particles ; i++)
	{
		_particles.push_back(Vector3d::Zero());
		_particles_with_weight.push_back(make_pair(Vector3d::Zero(),1));
	}

	_mean_scatter = 0.0;
	_std_scatter = 0.005;

	_memory_coefficient = 0.0;

	_coeff_friction = 0.0;
}

void ForceSpaceParticleFilter_weight_mem::update(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured)
{

	// resamplingLowVariance(motionUpdateAndWeighting(motion_control, force_control, velocity_measured, force_measured));
	resamplingLowVarianceProximityPenalty(motionUpdateAndWeighting(motion_control, force_control, velocity_measured, force_measured));

}

vector<pair<Vector3d, double>> ForceSpaceParticleFilter_weight_mem::motionUpdateAndWeighting(const Vector3d motion_control, const Vector3d force_control,
			const Vector3d velocity_measured, const Vector3d force_measured)
{
	Vector3d motion_control_normalized = Vector3d::Zero();
	Vector3d force_control_normalized = Vector3d::Zero();
	Vector3d measured_velocity_normalized = Vector3d::Zero();
	Vector3d measured_force_normalized = Vector3d::Zero();

	if(motion_control.norm() > 0.1)
	{
		motion_control_normalized = motion_control/motion_control.norm();
	}
	if(force_control.norm() > 0.1)
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

	// // add a particle in the force space is diemsion 2 or more
	// // augmented_particles.push_back(force_control_normalized);
	// int n_added_particle_force_space = 0;
	// if(_force_space_dimension == 2)
	// {
	// 	n_added_particle_force_space = 20;
	// 	Vector3d direction1 = force_control_normalized.cross(_motion_axis);
	// 	Vector3d direction2 = -direction1;
	// 	for(int i=0 ; i<n_added_particle_force_space/2 ; i++)
	// 	{
	// 		double alpha = (double) (i + 0.5) / (double)n_added_particle_force_space/2.0;
	// 		Vector3d new_particle1 = (1 - alpha) * direction1 + alpha * force_control_normalized;
	// 		Vector3d new_particle2 = (1 - alpha) * direction2 + alpha * force_control_normalized;
	// 		new_particle1.normalize();
	// 		new_particle2.normalize();
	// 		augmented_particles.push_back(new_particle1);
	// 		augmented_particles.push_back(new_particle2);
	// 	}
	// }


	// add particles in the direction of the motion control if there is no velocity in that direction
	double prob_add_particle = 0;
	if(motion_control.norm() - _coeff_friction * force_control.norm() > 0)
	{
		prob_add_particle = (1 - abs(tanh(100.0*velocity_measured.dot(motion_control_normalized)))) * (tanh(motion_control_normalized.dot(0.05*force_measured)));
	}
	// prob_add_particle *= tanh(motion_control.norm() - _coeff_friction * force_control.norm());
	// double prob_add_particle = (1 - abs(tanh(velocity_measured.dot(motion_control_normalized))));
	if(prob_add_particle < 0)
	{
		prob_add_particle = 0;
	}
	int n_added_particles = 1.5 * prob_add_particle * _n_particles;
	for(int i=0 ; i<n_added_particles ; i++)
	{
		double alpha = (double) (i + 0.5) / (double)n_added_particles; // add particles on the arc betwen the motion and force control
		Vector3d new_particle = (1 - alpha) * motion_control_normalized + alpha * force_control_normalized;
		new_particle.normalize();
		augmented_particles.push_back(new_particle);
	}


	int n_new_particles = 1 + n_added_particles;

	// prepare weights
	vector<pair<Vector3d, double>> augmented_weighted_particles;
	// double cumulative_weight = 0;

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
		double weight_velocity = 0.5;
		if(current_particle.norm() > 1e-3)
		{
			weight_velocity = 1 - abs(tanh(15.0*velocity_measured.dot(current_particle)));
		}

		// final weight
		double weight = weight_force * weight_velocity;
		if(i < _n_particles)
		{
			weight *= (1 - _memory_coefficient);
			weight += _memory_coefficient * _particles_with_weight[i].second;
		}
		

		// cumulative_weight += weight;
		augmented_weighted_particles.push_back(make_pair(current_particle, weight));
	}

	// for(int i=0 ; i< _n_particles + n_new_particles ; i++)
	// {
	// 	augmented_weighted_particles[i].second /= cumulative_weight;
	// }

	return augmented_weighted_particles;
}

void ForceSpaceParticleFilter_weight_mem::resamplingLowVariance(vector<pair<Vector3d, double>> augmented_weighted_particles)
{
	int n_augmented_weighted_particles = augmented_weighted_particles.size();
	vector<double> cumulative_weights;

	double sum_of_weights = 0;
	for(int i=0 ; i<n_augmented_weighted_particles ; i++)
	{
		sum_of_weights += augmented_weighted_particles[i].second;
		cumulative_weights.push_back(sum_of_weights);
	}

	for(int i=0 ; i<n_augmented_weighted_particles ; i++)
	{
		cumulative_weights[i] /= sum_of_weights;
	}

	double n_inv = 1.0/(double)_n_particles;
	double r = sampleUniformDistribution(0,n_inv);
	int k = 0;

	for(int i=0 ; i<_n_particles ; i++)
	{
		while(r > cumulative_weights[k])
		{
			k++;
		}
		_particles[i] = augmented_weighted_particles[k].first;

		_particles_with_weight[i].first = augmented_weighted_particles[k].first;
		_particles_with_weight[i].second = augmented_weighted_particles[k].second;

		r += n_inv;
	}
}

void ForceSpaceParticleFilter_weight_mem::resamplingLowVarianceProximityPenalty(vector<pair<Vector3d, double>> augmented_weighted_particles)
{
	int n_augmented_weighted_particles = augmented_weighted_particles.size();
	vector<double> cumulative_weights;

	// add penalty weight
	if(_force_space_dimension > 2)
	{
		vector<double> proximity_penalty_weights;
		for(int i=0 ; i<n_augmented_weighted_particles ; i++)
		{
			Vector3d current_particle = augmented_weighted_particles[i].first;

			// if(current_particle.norm() < 1e-3)
			// {
			// 	proximity_penalty_weights.push_back(1);
			// }
			// else
			// {
				double average_dist = 0;
				for(int j=0 ; j<n_augmented_weighted_particles ; j++)
				{
					average_dist += (current_particle - augmented_weighted_particles[j].first).norm();
				}
				average_dist /= (double) n_augmented_weighted_particles;

				double penalty_weight = 0.5;
				penalty_weight += average_dist;

				if(penalty_weight > 1)
				{
					penalty_weight = 1;
				}

				// cout << "i : " << i << endl;
				// cout << "particle : " << current_particle.transpose() << endl;
				// cout << "average_dist : " << average_dist << endl;
				// cout << "penalty_weight : " << penalty_weight << endl;
				// cout << endl;

				proximity_penalty_weights.push_back(penalty_weight);
			// }

			augmented_weighted_particles[i].second *= proximity_penalty_weights[i];
		}
	}
	

	double sum_of_weights = 0;
	for(int i=0 ; i<n_augmented_weighted_particles ; i++)
	{
		sum_of_weights += augmented_weighted_particles[i].second;
		cumulative_weights.push_back(sum_of_weights);
	}

	for(int i=0 ; i<n_augmented_weighted_particles ; i++)
	{
		cumulative_weights[i] /= sum_of_weights;
	}

	double n_inv = 1.0/(double)_n_particles;
	double r = sampleUniformDistribution(0,n_inv);
	int k = 0;

	for(int i=0 ; i<_n_particles ; i++)
	{
		while(r > cumulative_weights[k])
		{
			k++;
		}
		_particles[i] = augmented_weighted_particles[k].first;

		_particles_with_weight[i].first = augmented_weighted_particles[k].first;
		_particles_with_weight[i].second = augmented_weighted_particles[k].second;

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
