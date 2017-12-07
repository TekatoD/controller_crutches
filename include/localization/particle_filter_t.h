#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <iostream>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <log/trivial_logger_t.h>

#include "motion/pose2d_t.h"
#include "localization/line_t.h"
#include "field_map_t.h"


namespace drwn {
    // TODO: Refactor
    class particle_filter_t {
    public:
        static const int DEFAULT_PARTICLE_NUMBER = 100;
        static constexpr float DEFAULT_INIT_X = 0.0f;
        static constexpr float DEFAULT_INIT_Y = 0.0f;
        static constexpr float DEFAULT_INIT_THETA = 0.0f;
        static const int DEFAULT_RANDOM_PARTICLES = 0;
        static constexpr float DEFAULT_MIN_X = -3000.0f;
        static constexpr float DEFAULT_MIN_Y = -2000.0f;
        static constexpr float DEFAULT_MIN_THETA = -3.14f;
        static constexpr float DEFAULT_MAX_X = 3000.0f;
        static constexpr float DEFAULT_MAX_Y = 2000.0f;
        static constexpr float DEFAULT_MAX_THETA = 3.14f;
        static constexpr float DEFAULT_ODO_NOISE_ROT1 = 0.01f;
        static constexpr float DEFAULT_ODO_NOISE_TRANS = 50.0f;
        static constexpr float DEFAULT_ODO_NOISE_ROT2 = 0.01;
        static constexpr float DEFAULT_MEAS_NOISE_RANGE = 200.0f;
        static constexpr float DEFAULT_MEAS_NOISE_BEARING = 0.1f;
        static constexpr float DEFAULT_LOC_THRESHOLD_X = 200.0f;
        static constexpr float DEFAULT_LOC_THRESHOLD_Y = 200.0f;
        static constexpr float DEFAULT_LOC_THRESHOLD_THETA = 0.1f;

        using control_data = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >;
        using measurement_bundle = std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >;

        struct particle_t {
            pose2d_t pose;
            float weight{1.0f};
        };

        particle_filter_t();

        void initialize();

        void predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise);

        void correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise);

        void resample();

        std::vector<particle_t> get_particles() const { return m_particles; }

        // Pose mean
        pose2d_t get_pose_mean() const { return m_poseMean; }

        // Pose standard deviation
        pose2d_t get_pose_std_dev() const { return m_poseDev; }

        // Get pose of the particle with highest weight
        particle_t get_top_particle() const { return m_particles[m_topParticleIndex]; }


        void reset_pose(const pose2d_t& pose);

        void reset_pose(float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta);

        void reset_pose_to_field();

        static Eigen::Vector4f get_line_range_bearing(pose2d_t robot_pose, float x1, float y1, float x2, float y2);

        void calc_pose_mean_cov();

        void set_particle_number(int num_particles) {
            if (m_debug) LOG_DEBUG << "PARTICLE FILTER: num_particles = " << num_particles;
            m_config.num_particles = num_particles;
        }

        int get_particle_number() const { return m_config.num_particles; }

        void set_init_x(float init_x) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: init_x = " << init_x;
            m_config.init_x = init_x;
        }

        float get_init_x() const { return m_config.init_x; }

        void set_init_y(float init_y) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: init_y = " << init_y;
            m_config.init_y = init_y;
        }

        float get_init_y() const { return m_config.init_y; }

        void set_init_theta(float init_theta) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER init_theta = " << init_theta;
            m_config.init_theta = init_theta;
        }

        float get_init_theta() const { return m_config.init_theta; }

        void set_random_particles(int random_particles) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER random_particles = " << random_particles;
            m_config.random_particles = random_particles;
        }

        int get_random_particles() const { return m_config.random_particles; }

        void set_min_x(float min_x) {
            if (m_debug) LOG_DEBUG << "PARTICLE FILTER: min_x = " << min_x;
            m_config.min_x = min_x;
        }

        float get_min_x() const { return m_config.min_x; }

        void set_min_y(float min_y) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: min_y = " << min_y;
            m_config.min_y = min_y;
        }

        float get_min_y() const { return m_config.min_y; }

        void set_min_theta(float min_theta) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: min_theta = " << min_theta;
            m_config.min_theta = min_theta;
        }

        float get_min_theta() const { return m_config.min_theta; }

        void set_max_x(float max_x) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: max_x = " << max_x;

            m_config.max_x = max_x;
        }

        float get_max_x() const { return m_config.max_x; }

        void set_max_y(float max_y) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: max_y = " << max_y;
            m_config.max_y = max_y;
        }

        float get_max_y() const { return m_config.max_y; }

        void set_max_theta(float max_theta) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: max_theta = " << max_theta;
            m_config.max_theta = max_theta;
        }

        float get_max_theta() const { return m_config.max_theta; }

        void set_odo_noise_rot1(float odo_noise_rot1) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: odo_noise_rot1 = " << odo_noise_rot1;
            m_config.odo_noise_rot1 = odo_noise_rot1;
        }

        float get_odo_noise_rot1() const { return m_config.odo_noise_rot1; }

        void set_odo_noise_trans(float odo_noise_trans) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: odo_noise_trans = " << odo_noise_trans;
            m_config.odo_noise_trans = odo_noise_trans;
        }

        float get_odo_noise_trans() const { return m_config.odo_noise_trans; }

        void set_odo_noise_rot2(float odo_noise_rot2) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: odo_noise_rot2 = " << odo_noise_rot2;
            m_config.odo_noise_rot2 = odo_noise_rot2;
        }

        float get_odo_noise_rot2() const { return m_config.odo_noise_rot2; }

        void set_meas_noise_range(float meas_noise_range) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: meas_noise_range = " << meas_noise_range;
            m_config.meas_noise_range = meas_noise_range;
        }

        float get_meas_noise_range() const { return m_config.meas_noise_range; }

        void set_meas_noise_bearing(float meas_noise_bearing) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: meas_noise_bearing = " << meas_noise_bearing;
            m_config.meas_noise_bearing = meas_noise_bearing;
        }

        float get_meas_noise_bearing() const { return m_config.meas_noise_bearing; }

        float get_loc_threshold_x() const { return m_config.loc_threshold_x; }

        void set_loc_threshold_x(float x_dev) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: loc_threshold_x = " << x_dev;
            m_config.loc_threshold_x = x_dev;
        }

        float get_loc_threshold_y() const { return m_config.loc_threshold_y; }

        void set_loc_threshold_y(float y_dev) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: loc_threshold_y = " << y_dev;
            m_config.loc_threshold_y = y_dev;
        }

        float get_loc_threshold_theta() const { return m_config.loc_threshold_theta; }

        void set_loc_threshold_theta(float theta_dev) {
            if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: loc_threshold_theta = " << theta_dev;
            m_config.loc_threshold_theta = theta_dev;
        }

        bool is_localized() const;

        bool is_debug_enabled() const;

        void enable_debug(bool debug);

        float sample_normal_distribution(float variance);

        pose2d_t odometry_sample(pose2d_t pose, Eigen::Vector3f command, Eigen::Vector3f noise);

        Eigen::Vector3f get_odometry_command(pose2d_t prevPose, pose2d_t currPose);

    private:
        bool m_debug {false};
        bool m_localized {false};
        struct config_t {
            int num_particles;
            float init_x, init_y, init_theta;
            int random_particles;
            float min_x, min_y, min_theta;
            float max_x, max_y, max_theta;
            float odo_noise_rot1, odo_noise_trans, odo_noise_rot2;
            float meas_noise_range, meas_noise_bearing;
            float loc_threshold_x, loc_threshold_y, loc_threshold_theta;

            config_t()
                    : num_particles(DEFAULT_PARTICLE_NUMBER),
                      init_x(DEFAULT_INIT_X), init_y(DEFAULT_INIT_Y), init_theta(DEFAULT_INIT_THETA),
                      random_particles(DEFAULT_RANDOM_PARTICLES),
                      min_x(DEFAULT_MIN_X), min_y(DEFAULT_MIN_Y), min_theta(DEFAULT_MIN_THETA),
                      max_x(DEFAULT_MAX_X), max_y(DEFAULT_MAX_Y), max_theta(DEFAULT_MAX_THETA),
                      odo_noise_rot1(DEFAULT_ODO_NOISE_ROT1), odo_noise_trans(DEFAULT_ODO_NOISE_TRANS), odo_noise_rot2(DEFAULT_ODO_NOISE_ROT2),
                      meas_noise_range(DEFAULT_MEAS_NOISE_RANGE), meas_noise_bearing(DEFAULT_MEAS_NOISE_BEARING),
                      loc_threshold_x(DEFAULT_LOC_THRESHOLD_X), loc_threshold_y(DEFAULT_LOC_THRESHOLD_Y), loc_threshold_theta(DEFAULT_LOC_THRESHOLD_THETA)
            {
            }
        } m_config;

        std::vector<particle_t> m_particles;

        pose2d_t m_poseMean, m_poseDev;
        std::size_t m_topParticleIndex {0};
        void init_particles(const pose2d_t& pose, int num_particles);

        void init_particles(float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta,
                            int num_particles);

        void low_variance_resampling();

        void check_if_localized();

        std::tuple<field_map_t::line_type_t, point2d_t>
        calc_expected_measurement(float rx, float ry, float rtheta, float measured_range, float measured_bearing);
    };

}


#endif