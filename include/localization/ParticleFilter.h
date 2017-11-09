#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <iostream>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>

#include "minIni.h"
#include "Pose2D.h"
#include "localization/LocalizationUtil.h"

using namespace Robot;

namespace Localization {
    
class ParticleFilter {
public:
    static const int DEFAULT_PARTICLE_NUMBER = 100;
    
    using control_data = std::vector<Eigen::Vector3f>;
    using measurement_bundle = std::vector<Eigen::Vector4f>;
    
    struct Particle {
        Pose2D pose;
        float weight;
    };

    ParticleFilter(minIni* ini);
    
    ParticleFilter(const ParticleFilter& pf) = delete;
    ParticleFilter(ParticleFilter&& pf) = delete;
    ParticleFilter& operator=(const ParticleFilter& pf) = delete;
    ParticleFilter& operator=(ParticleFilter&& pf) = delete;
    
    void LoadIniSettings(minIni* ini);
    
    void predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise);
    void correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise);
    void resample();
    
    std::vector<Particle> getParticles() const { return m_particles; }
    // Pose mean
    Pose2D getPoseMean() const { return m_poseMean; }
    // Pose covariance
    Pose2D getPoseCovariance() const { return m_poseCovariance; }
    // Get pose of the particle with highest weight
    Particle getTopParticle() const { return m_particles[m_topParticleIndex]; }
     
    void calc_pose_mean_cov();
    
    /* Util functions, place in separate class */
    float sample_normal_distribution(float variance);
    Pose2D odometry_sample(Pose2D pose, Eigen::Vector3f command, Eigen::Vector3f noise);
    Eigen::Vector3f get_odometry_command(Pose2D prevPose, Pose2D currPose);
    /* */
private:
    std::vector<Particle> m_particles;
    Pose2D m_poseMean, m_poseCovariance;
    std::size_t m_topParticleIndex;
    Localization::FieldMap m_fieldWorld;
    
    void init_particles(const Pose2D& pose, int num_particles);
    void init_particles(float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta, int num_particles);
    
    void low_variance_resampling();
    std::tuple<FieldMap::LineType, Point2D> calc_expected_measurement(float rx, float ry, float rtheta, float measured_range, float measured_bearing);
};

}


#endif