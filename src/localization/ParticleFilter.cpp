#include "localization/ParticleFilter.h"

using namespace Localization;

ParticleFilter::ParticleFilter(const Robot::Pose2D& pose, int num_particles)
{
    init_particles(pose, num_particles);
}

ParticleFilter::ParticleFilter(int num_particles)
{
    init_particles(Robot::Pose2D(0, 0, 0), num_particles);
}

void ParticleFilter::predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise)
{
    
}

void ParticleFilter::correct(const measurement_data& measurements, const Eigen::Vector3f& noise)
{
    
}

void ParticleFilter::init_particles(const Robot::Pose2D& pose, int num_particles)
{
    
}

void ParticleFilter::low_variance_resampling()
{
}