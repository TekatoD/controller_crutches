//
// Created by akovalev on 16.11.17.
//

#include <localization/localization_t.h>
#include <opencv2/core/eigen.hpp>
// TODO: getter setter for head
#include <motion/modules/head_t.h>
#include <motion/kinematics_t.h>
#include <motion/motion_status_t.h>

using namespace drwn;

localization_t* localization_t::get_instance() {
    static localization_t instance;
    return &instance;
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

    if (m_debug) {
        LOG_DEBUG << "LOCALIZATION: m_old_pose = " << m_old_pose;
        LOG_DEBUG << "LOCALIZATION: m_current_pose = " << m_current_pose;
        LOG_DEBUG << "LOCALIZATION: odometry_command_rot1 = " << odometry_command(0);
        LOG_DEBUG << "LOCALIZATION: odometry_command_trans = " << odometry_command(1);
        LOG_DEBUG << "LOCALIZATION: odometry_command_rot2 = " << odometry_command(2);
    }

    // Move particles according to odometry command with addded noise
    Eigen::Vector3f movement_noise = {
            m_particle_filter->get_odo_noise_rot1(),
            m_particle_filter->get_odo_noise_trans(),
            m_particle_filter->get_odo_noise_rot2()
    };
    m_particle_filter->predict(odometry_command, movement_noise);

    // Forward kinematics for head
    // TODO: add  constants to kinemaitcs_t
    float height_from_ground = kinematics_t::LEG_LENGTH + 122.2f + 50.5f + kinematics_t::CAMERA_OFFSET_Z;
    float head_pan = head->get_pan_angle() * (M_PI / 180.0f);
    float head_tilt = head->get_tilt_angle() * (M_PI / 180.0f);

    float ankle_pitch = motion_status_t::current_joints.get_radian(joint_data_t::ID_L_ANKLE_PITCH);
    float ankle_roll = motion_status_t::current_joints.get_radian(joint_data_t::ID_L_ANKLE_ROLL);
    float knee_pitch = motion_status_t::current_joints.get_radian(joint_data_t::ID_L_KNEE);
    // TODO: Check correctness
    float thigh_pitch = motion_status_t::current_joints.get_radian(joint_data_t::ID_L_HIP_PITCH);
    float thigh_roll = motion_status_t::current_joints.get_radian(joint_data_t::ID_L_HIP_ROLL);
    float pelvis = motion_status_t::current_joints.get_radian(joint_data_t::ID_L_HIP_YAW);

    // HEAD TRANSFORM
    matrix4x4f_t head_transform_eigen;
    cv::Mat head_transform;

    kinematics_t::compute_head_forward_kinematics(head_transform_eigen, head_pan, head_tilt);
    cv::eigen2cv(head_transform_eigen, head_transform);

    // LEFT LEG TRANSFORM
    matrix4x4f_t left_leg_transform_eigen;
    cv::Mat left_leg_transform;

    kinematics_t::compute_leg_forward_kinematics(left_leg_transform_eigen, pelvis, thigh_roll, thigh_pitch,
                                                 knee_pitch, ankle_pitch, ankle_roll);
    cv::eigen2cv(left_leg_transform_eigen, left_leg_transform);

    // Extract head rotation matrix and translation vector for projecting lines
    cv::Mat head_rotation_matrix = head_transform(cv::Range(0, 3), cv::Range(0, 3));
    // Translation is in mm
    cv::Mat head_translation_vector = head_transform(cv::Range(0, 3), cv::Range(3, 4));
    //head_translation_vector.at<float>(2, 0) += height_from_ground;

    // Extract leg translation vector for calculating correct height from ground to head
    cv::Mat leg_translation_vector = left_leg_transform(cv::Range(0, 3), cv::Range(3, 4));
    leg_translation_vector.at<float>(2, 0) += 122.2f + 50.5f + kinematics_t::CAMERA_OFFSET_Z;
    head_translation_vector += leg_translation_vector;

    if (m_debug) LOG_DEBUG << "LOCALIZATION: head_translation_vector z = " << head_translation_vector.at<float>(2, 0);

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

        gp1 = m_camera_projection.image_to_world_explicit(mp1, head_rotation_matrix, head_translation_vector,
                                                          m_camera_params.width, m_camera_params.height,
                                                          m_camera_params.fov);
        gp2 = m_camera_projection.image_to_world_explicit(mp2, head_rotation_matrix, head_translation_vector,
                                                          m_camera_params.width, m_camera_params.height,
                                                          m_camera_params.fov);
        float x1 = gp1.at<float>(0, 0);
        // change direction of get_y axis
        float y1 = -1*gp1.at<float>(1, 0);

        float x2 = gp2.at<float>(0, 0);
        float y2 = -1*gp2.at<float>(1, 0);

        // Ignore lines that are projected too far (Like goal keeper gate lines)
        if ((x1 < 0.0f) || (x2 < 0.0f)) {
            continue;
        }

        Eigen::Vector4f lineRangeBearing = particle_filter_t::get_line_range_bearing(m_current_pose, x1, y1, x2, y2);

        rangeBearingData.push_back(lineRangeBearing);
    }

    Eigen::Vector3f measurement_noise = {
            m_particle_filter->get_meas_noise_range(),
            m_particle_filter->get_meas_noise_bearing(),
            0.0f
    };
    m_particle_filter->correct(rangeBearingData, measurement_noise);
    m_particle_filter->resample();

    auto const& pose_mean = m_particle_filter->get_pose_mean();
    auto const& pose_deviation = m_particle_filter->get_pose_std_dev();

    float x_dev = pose_deviation.get_x();
    float y_dev = pose_deviation.get_y();
    float theta_dev = pose_deviation.get_theta();

    m_localized = x_dev <= m_particle_filter->get_loc_threshold_x() &&
                  y_dev <= m_particle_filter->get_loc_threshold_y() &&
                  theta_dev <= m_particle_filter->get_loc_threshold_theta();

    if (m_debug)
    {
        LOG_DEBUG << "PARTICLE FILTER: Pose mean: " << pose_mean;
        LOG_DEBUG << "PARTICLE_FILTER: Pose std dev: " << pose_deviation;
    }

    m_old_pose = m_current_pose;
}

void localization_t::set_current_pose(const pose2d_t& current_pose)
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

void localization_t::set_pose_approximate_area(const pose2d_t& min_pose, const pose2d_t& max_pose)
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

void localization_t::set_movement_noise(float rot1, float trans, float rot2) {
    assert(m_particle_filter != nullptr);

    if (m_debug) { LOG_INFO << "LOCALIZATION: movement_noise = {" << rot1 << ", " << trans << ", " << rot2 << "}"; }
    m_particle_filter->set_odo_noise_rot1(rot1);
    m_particle_filter->set_odo_noise_trans(trans);
    m_particle_filter->set_odo_noise_rot2(rot2);
}

void localization_t::set_measurement_noise(float range, float bearing) {
    assert(m_particle_filter != nullptr);
    if (m_debug) { LOG_INFO << "LOCALIZATION: measurement_noise = {" << range << ", " << bearing << "}"; }
    m_particle_filter->set_meas_noise_range(range);
    m_particle_filter->set_meas_noise_bearing(bearing);
}

void localization_t::set_localization_threshold(float x_dev, float y_dev, float theta_dev) {
    assert(m_particle_filter != nullptr);

    m_particle_filter->set_loc_threshold_x(x_dev);
    m_particle_filter->set_loc_threshold_y(y_dev);
    m_particle_filter->set_loc_threshold_theta(theta_dev);
}

bool localization_t::is_localized() const {
    return m_localized;
}




