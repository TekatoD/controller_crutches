#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "Pose2D.h"

using namespace Robot;

namespace Localization {
    
class ParticleFilter {
public:
    static const int DEFAULT_PARTICLE_NUMBER = 100;
    
    using control_data = std::vector<Eigen::Vector3f>;
    using measurement_bundle = std::vector<Eigen::Vector3f>;
    using measurement_data = std::vector<measurement_bundle>;
    using world_data  = std::vector<Eigen::Vector3f>; 
    
    struct Particle {
        Pose2D pose;
        float weight;
    };

    ParticleFilter(const Pose2D& pose, int num_particles = DEFAULT_PARTICLE_NUMBER);
    ParticleFilter(int num_particles = DEFAULT_PARTICLE_NUMBER);
    
    ParticleFilter(const ParticleFilter& pf) = delete;
    ParticleFilter(ParticleFilter&& pf) = delete;
    ParticleFilter& operator=(const ParticleFilter& pf) = delete;
    ParticleFilter& operator=(ParticleFilter&& pf) = delete;
    
    void predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise);
    void correct(const measurement_data& measurements, const Eigen::Vector3f& noise);
     
    /* Util functions, place in separate class */
    float sample_normal_distribution(float variance);
    Pose2D odometry_sample(Pose2D pose, Eigen::Vector3f command, Eigen::Vector3f noise);
    /* */
private:
    std::vector<Particle> m_particles;
    
    void init_particles(const Pose2D& pose, int num_particles);
    
    void resample() { low_variance_resampling(); }
    void low_variance_resampling();
};

}


#endif