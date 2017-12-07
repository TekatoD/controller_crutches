//
// Created by akovalev on 16.11.17.
//

#pragma once

#include <localization/line_t.h>
#include <localization/particle_filter_t.h>
#include <vision/vision_utils.h>

namespace drwn {
    class localization_t {
    public:
        static localization_t* get_instance();

        // Motion data
        void set_pose_shift(pose2d_t pose_shift);
        // Sensor data
        void set_lines(std::vector<cv::Vec4i> lines);

        // Update based on motion and sensor data
        void update();

        void set_camera_parameters(vision_utils::camera_parameters_t camera_params);

        vision_utils::camera_parameters_t get_camera_parameters();

        // Resets PF particles to current pose
        void reset_current_pose(const pose2d_t &current_pose);

        // Resets PF particles to area
        void reset_pose_approximate_area(const pose2d_t &min_pose, const pose2d_t &max_pose);

        // Resets PF particles to area around pose with radius
        void reset_pose_approximate_area_around(const pose2d_t &pose, const pose2d_t &radius);

        // Resets PF to field area
        void reset_pose_approximate_area_to_field();

        void set_particle_filter(particle_filter_t* pf_ptr);
        particle_filter_t* get_particle_filter() const;

        void set_localization_threshold(float x_dev, float y_dev, float theta_dev);
        bool is_localized() const;

        // Set movement noise according to odometry motion model
        void set_movement_noise(float rot1, float trans, float rot2);

        // Set measurement noise according to range-bearing measurement model
        void set_measurement_noise(float range, float bearing);

        pose2d_t get_calculated_pose_mean();
        pose2d_t get_calculated_pose_std_dev();
        //
    private:
        localization_t() = default;

        bool m_debug {false};
    public:
        bool is_debug_enabled() const;

        void enable_debug(bool debug);

    private:
        pose2d_t m_old_pose, m_current_pose;
        std::vector<cv::Vec4i> m_current_lines;

        particle_filter_t* m_particle_filter;

        // TODO: camera_params move to hw
        vision_utils::camera_parameters_t m_camera_params;
        vision_utils::camera_projection_t m_camera_projection;
    };
}
