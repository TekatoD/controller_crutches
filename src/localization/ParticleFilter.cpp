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
    int random_particles = ini->geti("ParticleFilter", "random_particles", 0);
    
    if (random_particles > 0) {
        float min_x, min_y, min_theta, max_x, max_y, max_theta;
        min_x = ini->getf("ParticleFilter", "min_x");
        min_y = ini->getf("ParticleFilter", "min_y");
        min_theta = ini->getf("ParticleFilter", "min_theta");
        max_x = ini->getf("ParticleFilter", "max_x");
        max_y = ini->getf("ParticleFilter", "max_y");
        max_theta = ini->getf("ParticleFilter", "max_theta");
        
        init_particles(min_x, max_x, min_y, max_y, min_theta, max_theta, num_particles);
    } else {
        init_particles(Robot::Pose2D(poseX, poseY, poseTheta), num_particles);
    }
}

void ParticleFilter::predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise)
{
    for (auto& particle : m_particles) {
        particle.pose = odometry_sample(particle.pose, command, noise);
    }
}

void ParticleFilter::correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise)
{
    float weight_normalizer = 0.0f;
    for (auto& particle : m_particles) {
        if (measurements.size() < 1) {
            weight_normalizer = 1.0f;
            break;
        }
        
        float rx, ry, rtheta, vrange, vbearing;
        rx = particle.pose.X(); ry = particle.pose.Y(); rtheta = particle.pose.Theta();
        
        vrange = noise(0); vbearing = noise(1);
        
        //Eigen::MatrixXf Qt = Eigen::MatrixXf::Identity(measurements.size()*2, measurements.size()*2);
        //Qt = Qt * 0.1f;
        Eigen::Matrix2f Qt;
        Qt << vrange, 0.0f, 0.0f, vbearing;
        
        //Eigen::MatrixXf Zdiff(measurements.size()*2, 1);
        //int z_counter = 0;
        float lid, measured_range, measured_bearing, lx, ly, dx, dy;
        
        float new_weight = particle.weight;
        for (const auto& reading : measurements) {
            // Received measurement
            // Id is unknown for us
            lid = reading(0); measured_range = reading(1); measured_bearing = reading(2);
            Eigen::Vector2f z_measured = {measured_range, measured_bearing};
            
            // Predicted measurement
            // Cast a line from robot position with same bearing
            float epx, epy;
            // Need to normalize angles everywhere
            Pose2D normalizer(0.0, 0.0, measured_bearing + rtheta);
            epx = rx + FieldMap::MAX_DIST * cos(normalizer.Theta());
            epy = ry + FieldMap::MAX_DIST * sin(normalizer.Theta());
            
            FieldMap::LineType ret_type;
            Point2D isec_point;
            // Accept intersections with distance from robot to intersection point greater than minDist 
            auto isec_res = m_fieldWorld.IntersectWithField(
                Line(rx, ry, epx, epy), (measured_range - measured_range*0.25f)
            );
            ret_type = std::get<0>(isec_res);
            isec_point = std::get<1>(isec_res);
            
            if (ret_type == FieldMap::LineType::NONE) {
                continue;
            }
            
            lx = isec_point.X; ly = isec_point.Y;
            dx = lx - rx;
            dy = ly - ry;
            float expected_range = std::sqrt(dx*dx + dy*dy);
            
            if (expected_range > FieldMap::MAX_DIST) {
                continue;
            }
        
            normalizer.setTheta(std::atan2(dy, dx) - rtheta);
            float expected_bearing = normalizer.Theta();
            
            normalizer.setTheta(expected_bearing - measured_bearing);
            Eigen::Vector2f diff = {
                expected_range - measured_range,
                normalizer.Theta()
            };
            
            float det = (2.0f * M_PI * Qt).determinant();
            float temp = (diff.transpose() * Qt.inverse() * diff)(0);
            new_weight *= det * std::exp((-1.0f / 2.0f) * temp);
            
            if (std::isnan(new_weight)) {
                new_weight = 0.0f;
            }
            
            //Zdiff.block(z_counter, 0, 2, 1) = diff;
            //z_counter += 2;
        }
        
        weight_normalizer = weight_normalizer + new_weight;
        particle.weight = new_weight;
    }
    
    // While debugging
    if (weight_normalizer < 0.00001) {
        std::cout << "!!! weight normalizer is too close to 0 !!!" << std::endl;
    }
    
    float highestWeight = 0.0f;
    std::size_t highestWeightIndex = 0;
    for (std::size_t index = 0; index < m_particles.size(); index++) {
        Particle& particle = m_particles[index];
        
        if (fabs(weight_normalizer) < 0.00001 || std::isnan(weight_normalizer) || std::isnan(particle.weight)) {
            // Reset all weights
            particle.weight = 1.0f / m_particles.size();
        } else {
            particle.weight = particle.weight / weight_normalizer;
            
        }
        
        if (particle.weight > highestWeight) {
            highestWeight = particle.weight;
            highestWeightIndex = index;
        }
    }
    m_topParticleIndex = highestWeightIndex;
}

void ParticleFilter::resample()
{
    low_variance_resampling();
    calc_pose_mean_cov();
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
    Particle new_particle;
    for (int j = 0; j < m_particles.size(); j++) {
       float U = r + (j-1) * Jinv; 
       while (U > c) {
           i++;
           c += m_particles[i].weight;
       }
       new_particle.weight = 1.0f / m_particles.size();
       new_particle.pose = m_particles[i].pose;
       new_particles.push_back(new_particle);
    }
    
    if (new_particles.size() != m_particles.size()) {
        throw std::length_error("Length error in low_variance_resampling");
    }
    
    m_particles.swap(new_particles);
}

void ParticleFilter::calc_pose_mean_cov()
{
    // Only the best particles remain after resampling
    // Weights are uniform after resampling
    
    Pose2D meanAccum;
    float pw, px, py, ptheta;
    for (const auto& particle : m_particles) {
        
        pw = particle.weight;
        px = particle.pose.X();
        py = particle.pose.Y();
        ptheta = particle.pose.Theta();
        
        meanAccum += Pose2D(px*pw, py*pw, ptheta*pw);
    }
    
    Pose2D covAccum, angleNormalizer;
    float mx, my, mtheta;
    mx = meanAccum.X();
    my = meanAccum.Y();
    mtheta = meanAccum.Theta();
    for (const auto& particle : m_particles) {
        pw = particle.weight;
        px = particle.pose.X();
        py = particle.pose.Y();
        ptheta = particle.pose.Theta();
        
        angleNormalizer.setTheta(ptheta-mtheta);
        covAccum += Pose2D(pw*pow(px-mx, 2), pw*pow(py-my, 2), pw*pow(angleNormalizer.Theta(), 2));
    }
    
    m_poseMean = meanAccum;
    m_poseCovariance = covAccum;
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

void ParticleFilter::init_particles(float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta, int num_particles)
{
    float defaultWeight = 1.0f / num_particles;
    m_topParticleIndex = 0;
    
    m_particles.resize(num_particles);
    
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<> dist_x(min_x, max_x);
    std::uniform_real_distribution<> dist_y(min_y, max_y);
    std::uniform_real_distribution<> dist_theta(min_theta, max_theta);
    
    for (auto& particle : m_particles) {
        particle.pose = Pose2D(dist_x(generator), dist_y(generator), dist_theta(generator));
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

Eigen::Vector3f ParticleFilter::get_odometry_command(Pose2D prevPose, Pose2D currPose)
{
    float rot1, trans, rot2;
    float dx, dy;
    dx = currPose.X() - prevPose.X();
    dy = currPose.Y() - prevPose.Y();
    
    trans = sqrt(dx*dx + dy*dy);
    Pose2D normalizer(0.0f, 0.0f, atan2(dy, dx) - prevPose.Theta());
    rot1 = normalizer.Theta();
    normalizer.setTheta(currPose.Theta() - prevPose.Theta() - rot1);
    rot2 = normalizer.Theta();
    
    return Eigen::Vector3f {
        rot1,
        trans,
        rot2
    };
}