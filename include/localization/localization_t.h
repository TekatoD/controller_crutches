//
// Created by akovalev on 16.11.17.
//

#pragma once

#include <localization/localization_util_t.h>
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

        void update();

        // Resets PF particles to current pose
        void set_current_pose(pose2d_t current_pose);

        void set_field(field_map_t field);
        field_map_t get_field() const;

        void set_camera_parameters(vision_utils::camera_parameters_t camera_params);
        vision_utils::camera_parameters_t get_camera_parameters();

        // Resets PF particles to area
        void set_pose_approximate_area(pose2d_t min_pose, pose2d_t max_pose);

        // Resets PF to field area
        void set_pose_approximate_area_to_field();

        void set_particle_filter(particle_filter_t* pf_ptr);
        particle_filter_t* get_particle_filter() const;

        pose2d_t get_calculated_pose_mean();
        pose2d_t get_calculated_pose_std_dev();
        //
    private:
        localization_t();

        bool m_debug;
    public:
        bool is_debug_enabled() const;

        void enable_debug(bool debug);

    private:
        particle_filter_t* m_particle_filter;

        // TODO: camera_params move to hw
        vision_utils::camera_parameters_t m_camera_params;
        vision_utils::camera_projection_t m_camera_projection;
    };
}
