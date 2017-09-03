#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <iostream>
#include <vector>
#include <map>
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

    ParticleFilter(const Pose2D& pose, world_data world, int num_particles = DEFAULT_PARTICLE_NUMBER);
    ParticleFilter(world_data world, int num_particles = DEFAULT_PARTICLE_NUMBER);
    
    ParticleFilter(const ParticleFilter& pf) = delete;
    ParticleFilter(ParticleFilter&& pf) = delete;
    ParticleFilter& operator=(const ParticleFilter& pf) = delete;
    ParticleFilter& operator=(ParticleFilter&& pf) = delete;
    
    void predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise);
    void correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise);
    void resample();
    
    // TODO: Per-particle predict and correct functions
    
    std::vector<Particle> getParticles() const { return m_particles; }
    std::map<int, Eigen::Vector2f> getWorld() const { return m_world; }
     
    /* Util functions, place in separate class */
    float sample_normal_distribution(float variance);
    Pose2D odometry_sample(Pose2D pose, Eigen::Vector3f command, Eigen::Vector3f noise);
    /* */
private:
    std::vector<Particle> m_particles;
    std::map<int, Eigen::Vector2f> m_world;
    
    void init_particles(const Pose2D& pose, int num_particles);
    void prepare_world(const world_data& world);
    
    void low_variance_resampling();
};

}


#endif