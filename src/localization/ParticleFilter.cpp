#include <chrono>
#include <random>
#include "localization/ParticleFilter.h"

using namespace Localization;
using namespace Robot;

ParticleFilter::ParticleFilter(const Robot::Pose2D& pose, world_data world, int num_particles)
{
    init_particles(pose, num_particles);
    prepare_world(world);
}

ParticleFilter::ParticleFilter(world_data world, int num_particles)
{
    init_particles(Robot::Pose2D(0, 0, 0), num_particles);
    prepare_world(world);
}

void ParticleFilter::predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise)
{
    for (auto itr = m_particles.begin(); itr != m_particles.end(); itr++) {
        Particle p = (*itr);
        p.pose = odometry_sample(p.pose, command, noise);
        (*itr) = p;
    }
}

void ParticleFilter::correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise)
{
    
}

void ParticleFilter::init_particles(const Pose2D& pose, int num_particles)
{
    float defaultWeight = 1.0f / num_particles;
    
    m_particles.resize(num_particles);
    for (int i = 0; i < m_particles.size(); i++) {
        m_particles[i].pose = pose;
        m_particles[i].weight = defaultWeight;
    }
}

void ParticleFilter::prepare_world(const world_data& world)
{
    for (const auto& landmark : world) {
        Eigen::Vector2f lpos = {landmark(1), landmark(2)};
        m_world[landmark(0)] = lpos;
    }
}

void ParticleFilter::low_variance_resampling()
{
}

float ParticleFilter::sample_normal_distribution(float variance)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<float> normal(0.0f, variance);
    
    return normal(generator);
}

Pose2D ParticleFilter::odometry_sample(Pose2D pose, Eigen::Vector3f command, Eigen::Vector3f noise)
{
    float rot1, trans, rot2;
    rot1 = command(0); trans = command(1); rot2 = command(2);
    
    float r1_noise, t_noise, r2_noise;
    r1_noise = noise(0); t_noise = noise(1), r2_noise = noise(2);
    
    pose.normalizeTheta();
    float theta_old = pose.Theta();
    
    float rot1_h, trans_h, rot2_h;
    rot1_h = rot1 - sample_normal_distribution(r1_noise);
    trans_h = trans - sample_normal_distribution(t_noise);
    rot2_h = rot2 - sample_normal_distribution(r2_noise);
    
    Pose2D newPose;
    newPose.setTheta(theta_old + rot1_h);
    float normalized = newPose.Theta();
    
    newPose.setX(trans_h * std::cos(normalized));
    newPose.setY(trans_h * std::sin(normalized));
    newPose.setTheta(rot1_h + rot2_h);
    
    return pose + newPose;
}