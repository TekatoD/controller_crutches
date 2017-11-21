//
// Created by akovalev on 16.11.17.
//

#include <localization/localization_t.h>
#include <opencv2/core/eigen.hpp>
// TODO: getter setter for head
#include <motion/modules/head_t.h>
#include <motion/kinematics_t.h>

using namespace drwn;

localization_t* localization_t::get_instance() {
    static localization_t instance;
    return &instance;
}

localization_t::localization_t()
{
    // TODO: load init values from config (make particle filter do it)
    // Odometry command noise:
    // rotation1, translation, rotation2
    m_movement_noise = {0.01f, 50.0f, 0.01f};
    // Range-bearing measurement noise
    // range, bearing, (3rd not used)
    m_measurement_noise = {600.0f, 0.05f, 0.0f};
}

void localization_t::set_pose_shift(pose2d_t pose_shift)
{
    m_old_pose = m_current_pose;
    m_current_pose = pose_shift;
}

void localization_t::set_lines(std::vector<cv::Vec4i> lines)
{
    m_current_lines = std::move(lines);
}

void localization_t::update()
{
    assert(m_particle_filter != nullptr);

    auto head = head_t::get_instance();
    // Calculate odometry command from pose change
    Eigen::Vector3f odometry_command = m_particle_filter->get_odometry_command(m_old_pose, m_current_pose);
    // Move particles according to odometry command with addded noise
    m_particle_filter->predict(odometry_command, m_movement_noise);

    // Forward kinematics for head
    float height_from_ground = kinematics_t::LEG_LENGTH + 122.2f + 50.5f + kinematics_t::CAMERA_OFFSET_Z;
    float head_pan = head->get_pan_angle() * (M_PI / 180.0f);
    float head_tilt = head->get_tilt_angle() * (M_PI / 180.0f);

    matrix4x4f_t head_transform_eigen;
    cv::Mat head_transform;

    kinematics_t::compute_head_forward_kinematics(head_transform_eigen, head_pan, head_tilt);
    cv::eigen2cv(head_transform_eigen, head_transform);

    cv::Mat R = head_transform(cv::Range(0, 3), cv::Range(0, 3));
    // Translation is in mm
    cv::Mat t = head_transform(cv::Range(0, 3), cv::Range(3, 4));
    //t /= 1000.0f;
    t.at<float>(2, 0) += height_from_ground;

    // Collect measurements from lines
    particle_filter_t::measurement_bundle rangeBearingData;
    for (auto& line : m_current_lines) {
        if (line[0] > m_camera_params.width || line[2] > m_camera_params.width ||
            line[1] > m_camera_params.height || line[3] > m_camera_params.height) {
            // why
            continue;
        }

        cv::Mat mp1, mp2, gp1, gp2;
        mp1 = (cv::Mat_<float>(3, 1) << line[0], line[1], 1);
        mp2 = (cv::Mat_<float>(3, 1) << line[2], line[3], 1);

        gp1 = m_camera_projection.image_to_world_explicit(mp1, R, t, m_camera_params.width, m_camera_params.height, m_camera_params.fov);
        gp2 = m_camera_projection.image_to_world_explicit(mp2, R, t, m_camera_params.width, m_camera_params.height, m_camera_params.fov);
        float x1 = gp1.at<float>(0, 0);
        // change direction of get_y axis
        float y1 = -1*gp1.at<float>(1, 0);

        float x2 = gp2.at<float>(0, 0);
        float y2 = -1*gp2.at<float>(1, 0);

        // Ignore lines that are projected  too far (Like goal keeper gate lines)
        if ((x1 < 0.0f) || (x2 < 0.0f)) {
            continue;
        }

        Eigen::Vector4f lineRangeBearing = particle_filter_t::get_line_range_bearing(m_current_pose, x1, y1, x2, y2);

        rangeBearingData.push_back(lineRangeBearing);
    }

    m_particle_filter->correct(rangeBearingData, m_measurement_noise);
    m_particle_filter->resample();

    if (m_debug)
    {
        LOG_INFO << "PARTICLE FILTER: Pose information start";
        LOG_INFO << "PARTICLE FILTER: Pose mean: " << m_particle_filter->get_pose_mean();
        LOG_INFO << "PARTICLE_FILTER: Pose std dev: " << m_particle_filter->get_pose_std_dev();
        LOG_INFO << "PARTICLE FILTER: Pose information end";
    }

    m_old_pose = m_current_pose;
}

void localization_t::set_current_pose(pose2d_t current_pose)
{
    assert(m_particle_filter != nullptr);
    m_particle_filter->reset_pose(current_pose);
}

void localization_t::set_camera_parameters(vision_utils::camera_parameters_t camera_params)
{
    m_camera_params = camera_params;
}

vision_utils::camera_parameters_t localization_t::get_camera_parameters()
{
    return m_camera_params;
}

void localization_t::set_pose_approximate_area(pose2d_t min_pose, pose2d_t max_pose)
{
    assert(m_particle_filter != nullptr);
    float min_x, min_y, min_theta, max_x, max_y, max_theta;
    min_x = min_pose.get_x(); min_y = min_pose.get_y(); min_theta = min_pose.get_theta();
    max_x = max_pose.get_x(); max_y = max_pose.get_y(); max_theta = max_pose.get_theta();
    m_particle_filter->reset_pose(min_x, max_x, min_y, max_y, min_theta, max_theta);
}

void localization_t::set_pose_approximate_area_to_field()
{
    assert(m_particle_filter != nullptr);
    m_particle_filter->reset_pose_to_field();
}

void localization_t::set_particle_filter(particle_filter_t* pf_ptr)
{
    assert(pf_ptr != nullptr);
    m_particle_filter = pf_ptr;
}

particle_filter_t* localization_t::get_particle_filter() const
{
    assert(m_particle_filter != nullptr);
    return m_particle_filter;
}

pose2d_t localization_t::get_calculated_pose_mean()
{
    assert(m_particle_filter != nullptr);
    return m_particle_filter->get_pose_mean();
}

pose2d_t localization_t::get_calculated_pose_std_dev()
{
    assert(m_particle_filter != nullptr);
    return m_particle_filter->get_pose_std_dev();
}

bool localization_t::is_debug_enabled() const {
    return m_debug;
}

void localization_t::enable_debug(bool debug) {
    assert(m_particle_filter != nullptr);
    m_debug = debug;
    m_particle_filter->enable_debug(debug);
}


