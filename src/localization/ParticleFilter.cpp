#include <cmath>
#include <chrono>
#include <random>
#include "localization/ParticleFilter.h"

using namespace Localization;
using namespace Robot;

ParticleFilter::ParticleFilter(minIni* ini)
{
    LoadIniSettings(ini);
}

void ParticleFilter::LoadIniSettings(minIni* ini)
{
    m_fieldWorld.LoadIniSettings(ini);
    
    float poseX = ini->getf("ParticleFilter", "init_x");
    float poseY = ini->getf("ParticleFilter", "init_y");
    float poseTheta = ini->getf("ParticleFilter", "init_theta");
    int num_particles = ini->geti("ParticleFilter", "particle_number", DEFAULT_PARTICLE_NUMBER); 
    
    init_particles(Robot::Pose2D(poseX, poseY, poseTheta), num_particles);
}

void ParticleFilter::predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise)
{
    for (auto& particle : m_particles) {
        particle.pose = odometry_sample(particle.pose, command, noise);
    }
}

void ParticleFilter::correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise)
{
    /*
    float normalizer = 0;
    for (auto& particle : m_particles) {
        float rx, ry, rtheta, vrange, vbearing;
        rx = particle.pose.X(); ry = particle.pose.Y(); rtheta = particle.pose.Theta();
        
        vrange = noise(0); vbearing = noise(1);
        
        // TODO: Create matrix from vector
        Eigen::MatrixXf Qt = Eigen::MatrixXf::Identity(measurements.size()*2, measurements.size()*2);
        Qt = Qt * 0.1;
        
        Eigen::MatrixXf Zdiff(measurements.size()*2, 1);
        int z_counter = 0;
        float lid, srange, sbearing, lx, ly, dx, dy;
        for (const auto& reading : measurements) {
            lid = reading(0); srange = reading(1); sbearing = reading(2);
            Eigen::Vector2f z_measured = {srange, sbearing};
            
            Eigen::Vector2f lm = m_world[lid];
            lx = lm(0); ly = lm(1);
            
            dx = lx - rx;
            dy = ly - ry;
            Eigen::Vector2f delta = { dx, dy };
            float q = delta.transpose() * delta;
            
            Pose2D normalizer(0, 0, std::atan2(dy, dx) - rtheta);
            Eigen::Vector2f z_expected = {
                std::sqrt(q),
                normalizer.Theta()
            };
            
            Eigen::Vector2f diff = z_expected - z_measured;
            normalizer.setTheta(diff(1));
            diff(1) = normalizer.Theta();
            
            Zdiff.block(z_counter, 0, 2, 1) = diff;
            z_counter += 2;
        }
        
        float det = (2 * M_PI * Qt).determinant();
        float denom = 1 / std::sqrt(det);
        
        float temp = (Zdiff.transpose() * Qt.inverse() * Zdiff)(0);
        float new_weight = denom * std::exp((-1.0f / 2.0f) * temp);
        
        normalizer = normalizer + new_weight;
        particle.weight = new_weight;
    }
    
    Pose2D meanAccum;
    float pw, px, py, ptheta;
    float highestWeight = 0.0f;
    std::size_t highestWeightIndex = 0;
    for (std::size_t index = 0; index < m_particles.size(); index++) {
        Particle& particle = m_particles[index];
        particle.weight = particle.weight / normalizer;
        pw = particle.weight;
        px = particle.pose.X();
        py = particle.pose.Y();
        ptheta = particle.pose.Theta();
        
        meanAccum += Pose2D(px*pw, py*pw, pw*ptheta);
        
        if (pw > highestWeight) {
            highestWeight = pw;
            highestWeightIndex = index;
        }
    }
    m_topParticleIndex = highestWeightIndex;
    
    Pose2D covAccum;
    float mx, my, mtheta;
    mx = meanAccum.X();
    my = meanAccum.Y();
    mtheta = meanAccum.Theta();
    for (auto& particle : m_particles) {
        pw = particle.weight;
        px = particle.pose.X();
        py = particle.pose.Y();
        ptheta = particle.pose.Theta();
        
        covAccum += Pose2D(pw*pow(px-mx, 2), pw*pow(py-my, 2), pw*pow(ptheta-mtheta, 2));
    }
    
    m_poseMean = meanAccum;
    m_poseCovariance = covAccum;
    */
}

void ParticleFilter::resample()
{
    low_variance_resampling();
}

void ParticleFilter::low_variance_resampling()
{
    std::vector<Particle> new_particles;
    
    float Jinv = 1.0f / m_particles.size();
    
    std::default_random_engine generator;
    std::uniform_real_distribution<float> dist(0.0, Jinv);
    float r = dist(generator);
    
    float c = m_particles[0].weight;
    int i = 0;
    for (int j = 0; j < m_particles.size(); j++) {
       float U = r + (j-1) * Jinv; 
       while (U > c) {
           i++;
           c += m_particles[i].weight;
       }
       new_particles.push_back(m_particles[i]);
    }
    
    if (new_particles.size() != m_particles.size()) {
        throw std::length_error("Length error in low_variance_resampling");
    }
    
    m_particles.swap(new_particles);
}

void ParticleFilter::init_particles(const Pose2D& pose, int num_particles)
{
    float defaultWeight = 1.0f / num_particles;
    m_topParticleIndex = 0;
    
    m_particles.resize(num_particles);
    for (auto& particle : m_particles) {
        particle.pose = pose;
        particle.weight = defaultWeight;
    }
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