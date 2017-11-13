#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <iostream>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>

#include "minIni.h"
#include "Pose2D.h"
#include "localization/localization_util_t.h"

using namespace Robot;

namespace localization {
    
class particle_filter_t {
public:
    static const int DEFAULT_PARTICLE_NUMBER = 100;
    
    using control_data = std::vector<Eigen::Vector3f>;
    using measurement_bundle = std::vector<Eigen::Vector4f>;
    
    struct particle_t {
        Pose2D pose;
        float weight;
    };

    particle_filter_t(minIni* ini);
    
    particle_filter_t(const particle_filter_t& pf) = delete;
    particle_filter_t(particle_filter_t&& pf) = delete;
    particle_filter_t& operator=(const particle_filter_t& pf) = delete;
    particle_filter_t& operator=(particle_filter_t&& pf) = delete;
    
    void load_ini_settings(minIni *ini);
    
    void predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise);
    void correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise);
    void resample();
    
    std::vector<particle_t> get_particles() const { return m_particles; }
    // Pose mean
    Pose2D get_pose_mean() const { return m_poseMean; }
    // Pose standard deviation
    Pose2D get_pose_std_dev() const { return m_poseDev; }
    // Get pose of the particle with highest weight
    particle_t get_top_particle() const { return m_particles[m_topParticleIndex]; }
     
    void calc_pose_mean_cov();
    
    /* Util functions, place in separate class */
    float sample_normal_distribution(float variance);
    Pose2D odometry_sample(Pose2D pose, Eigen::Vector3f command, Eigen::Vector3f noise);
    Eigen::Vector3f get_odometry_command(Pose2D prevPose, Pose2D currPose);
    /* */
private:
    std::vector<particle_t> m_particles;
    Pose2D m_poseMean, m_poseDev;
    std::size_t m_topParticleIndex;
    localization::field_map_t m_fieldWorld;
    
    void init_particles(const Pose2D& pose, int num_particles);
    void init_particles(float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta, int num_particles);
    
    void low_variance_resampling();
    std::tuple<field_map_t::line_type_t, Point2D> calc_expected_measurement(float rx, float ry, float rtheta, float measured_range, float measured_bearing);
};

}


#endif