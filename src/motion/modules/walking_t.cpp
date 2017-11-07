/*
 *   Walking.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <iostream>
#include <math/angle_tools.h>
#include <log/trivial_logger_t.h>
#include "hw/MX28_t.h"
#include "motion/motion_status_t.h"
#include "motion/kinematics_t.h"
#include "motion/modules/walking_t.h"

using namespace drwn;

walking_t::walking_t() {
    m_x_offset = -10;
    m_y_offset = 5;
    m_z_offset = 20;
    m_r_offset = 0;
    m_p_offset = 0;
    m_a_offset = 0;
    m_hip_pitch_offset = 13.0;
    m_period_time = 600;
    m_dsp_ratio = 0.1;
    m_step_fb_ratio = 0.28;
    m_z_move_amplitude = 40;
    m_y_swap_amplitude = 20.0;
    m_z_swap_amplitude = 5;
    m_pelvis_offset = 3.0;
    m_arm_swing_gain = 1.5;
    m_balance_knee_gain = 0.3;
    m_balance_ankle_pitch_gain = 0.9;
    m_balance_hip_roll_gain = 0.5;
    m_balance_ankle_roll_gain = 1.0;

    m_p_gain = joint_data_t::P_GAIN_DEFAULT;
    m_i_gain = joint_data_t::I_GAIN_DEFAULT;
    m_d_gain = joint_data_t::D_GAIN_DEFAULT;

    m_x_move_amplitude = 0;
    m_y_move_amplitude = 0;
    m_a_move_amplitude = 0;
    m_move_aim_on = false;
    m_balance_enable = true;

    m_odo_x = 0.0;
    m_odo_y = 0.0;
    m_odo_theta = 0.0;

    m_left_odo_x = 0.0;
    m_left_odo_y = 0.0;
    m_left_odo_theta = 0.0;

    m_right_odo_x = 0.0;
    m_right_odo_y = 0.0;
    m_right_odo_theta = 0.0;

    m_left_end = false;
    m_right_end = false;
    m_left_start = false;
    m_right_start = false;

    m_odo_x_factor = 2.0;
    m_odo_y_factor = 2.0;
    m_odo_a_factor = 0.8;

    joint.set_angle(joint_data_t::ID_R_SHOULDER_PITCH, -48.345f);
    joint.set_angle(joint_data_t::ID_L_SHOULDER_PITCH, 41.313f);
    joint.set_angle(joint_data_t::ID_R_SHOULDER_ROLL, -17.873f);
    joint.set_angle(joint_data_t::ID_L_SHOULDER_ROLL, 17.580f);
    joint.set_angle(joint_data_t::ID_R_ELBOW, 29.300f);
    joint.set_angle(joint_data_t::ID_L_ELBOW, -29.593f);

    joint.set_angle(joint_data_t::ID_HEAD_TILT, kinematics_t::EYE_TILT_OFFSET_ANGLE);

    joint.set_p_gain(joint_data_t::ID_R_SHOULDER_PITCH, 8);
    joint.set_p_gain(joint_data_t::ID_L_SHOULDER_PITCH, 8);
    joint.set_p_gain(joint_data_t::ID_R_SHOULDER_ROLL, 8);
    joint.set_p_gain(joint_data_t::ID_L_SHOULDER_ROLL, 8);
    joint.set_p_gain(joint_data_t::ID_R_ELBOW, 8);
    joint.set_p_gain(joint_data_t::ID_L_ELBOW, 8);
}

void walking_t::update_param_time() {
    if (m_debug) {
        LOG_TRACE << "WALKING: Updating time parameters";
    }
    m_cur_period_time = m_period_time;
    m_cur_dsp_ratio = m_dsp_ratio;
    m_cur_ssp_ratio = 1 - m_dsp_ratio;

    m_cur_x_swap_period_time = m_cur_period_time / 2;
    m_cur_x_move_period_time = m_cur_period_time * m_cur_ssp_ratio;
    m_cur_y_swap_period_time = m_cur_period_time;
    m_cur_y_move_period_time = m_cur_period_time * m_cur_ssp_ratio;
    m_cur_z_swap_period_time = m_cur_period_time / 2;
    m_cur_z_move_period_time = m_cur_period_time * m_cur_ssp_ratio / 2;
    m_cur_a_move_period_time = m_cur_period_time * m_cur_ssp_ratio;

    m_cur_ssp_time = m_cur_period_time * m_cur_ssp_ratio;
    m_cur_ssp_time_start_l = (1 - m_cur_ssp_ratio) * m_cur_period_time / 4;
    m_cur_ssp_time_end_l = (1 + m_cur_ssp_ratio) * m_cur_period_time / 4;
    m_cur_ssp_time_start_r = (3 - m_cur_ssp_ratio) * m_cur_period_time / 4;
    m_cur_ssp_time_end_r = (3 + m_cur_ssp_ratio) * m_cur_period_time / 4;

    m_cur_phase_time1 = (m_cur_ssp_time_end_l + m_cur_ssp_time_start_l) / 2;
    m_cur_phase_time2 = (m_cur_ssp_time_start_r + m_cur_ssp_time_end_l) / 2;
    m_cur_phase_time3 = (m_cur_ssp_time_end_r + m_cur_ssp_time_start_r) / 2;

    m_cur_pelvis_offset = m_pelvis_offset * MX28_t::RATIO_DEGREES2VALUE;
    m_cur_pelvis_swing = m_cur_pelvis_offset * 0.35f;
    m_cur_arm_swing_gain = m_arm_swing_gain;
}


void walking_t::update_param_move() {
    if (m_debug) {
        LOG_TRACE << "WALKING: Updating move parameters";
    }
    // Forward/Back
    m_cur_x_move_amplitude = m_x_move_amplitude;
    m_cur_x_swap_amplitude = m_x_move_amplitude * m_step_fb_ratio;

    // Right/Left
    m_cur_y_move_amplitude = m_y_move_amplitude / 2;
    if (m_cur_y_move_amplitude > 0)
        m_cur_y_move_amplitude_shift = m_cur_y_move_amplitude;
    else
        m_cur_y_move_amplitude_shift = -m_cur_y_move_amplitude;
    m_cur_y_swap_amplitude = m_y_swap_amplitude + m_cur_y_move_amplitude_shift * 0.04f;

    m_cur_z_move_amplitude = m_z_move_amplitude / 2;
    m_cur_z_move_amplitude_shift = m_cur_z_move_amplitude / 2;
    m_cur_z_swap_amplitude = m_z_swap_amplitude;
    m_cur_z_swap_amplitude_shift = m_cur_z_swap_amplitude;

    // Direction
    if (!m_move_aim_on) {
        m_cur_a_move_amplitude = radians(m_a_move_amplitude) / 2;
        if (m_cur_a_move_amplitude > 0)
            m_cur_a_move_amplitude_shift = m_cur_a_move_amplitude;
        else
            m_cur_a_move_amplitude_shift = -m_cur_a_move_amplitude;
    } else {
        m_cur_a_move_amplitude = -radians(m_a_move_amplitude) / 2;
        if (m_cur_a_move_amplitude > 0)
            m_cur_a_move_amplitude_shift = -m_cur_a_move_amplitude;
        else
            m_cur_a_move_amplitude_shift = m_cur_a_move_amplitude;
    }
}


void walking_t::update_param_balance() {
    if (m_debug) {
        LOG_TRACE << "WALKING: Updating balance parameters";
    }
    m_cur_x_offset = m_x_offset;
    m_cur_y_offset = m_y_offset;
    m_cur_z_offset = m_z_offset;
    m_cur_r_offset = radians(m_r_offset);
    m_cur_p_offset = radians(m_p_offset);
    m_cur_a_offset = radians(m_a_offset);
    m_cur_hip_pitch_offset = m_hip_pitch_offset * MX28_t::RATIO_DEGREES2VALUE;
}


void walking_t::initialize() {
    m_x_move_amplitude = 0;
    m_y_move_amplitude = 0;
    m_a_move_amplitude = 0;

    m_cur_body_swing_y = 0;
    m_cur_body_swing_z = 0;

    m_cur_x_swap_phase_shift = pi;
    m_cur_x_swap_amplitude_shift = 0;
    m_cur_x_move_phase_shift = half_pi;
    m_cur_x_move_amplitude_shift = 0;
    m_cur_y_swap_phase_shift = 0;
    m_cur_y_swap_amplitude_shift = 0;
    m_cur_y_move_phase_shift = half_pi;
    m_cur_z_swap_phase_shift = half_pi * 3;
    m_cur_z_move_phase_shift = half_pi;
    m_cur_a_move_phase_shift = half_pi;

    m_ctrl_running = false;
    m_real_running = false;
    m_time = 0;
    update_param_time();
    update_param_move();

    process();
}


void walking_t::start() {
    if (m_debug && !m_ctrl_running) {
        LOG_DEBUG << "WALKING: Gait was started";
    }
    m_ctrl_running = true;
    m_real_running = true;
}


void walking_t::stop() {
    if (m_debug) {
        LOG_DEBUG << "WALKING: stop signal was received";
    }
    m_ctrl_running = false;
}


bool walking_t::is_running() {
    return m_real_running;
}


void walking_t::process() {
    float x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
    float x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
    float x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
    float pelvis_offset_r, pelvis_offset_l;
    float angle[14], ep[12];
    float offset;
    constexpr float TIME_UNIT = motion_module_t::TIME_UNIT;
    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    constexpr int dir[14] = {-1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1};
    constexpr float initAngle[14] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -48.345f, 41.313f};
    int outValue[14];

    // Update walk parameters
    if (m_time == 0) {
        update_param_time();
        m_phase = PHASE0;
        if (!m_ctrl_running) {
            if (m_cur_x_move_amplitude == 0 && m_cur_y_move_amplitude == 0 && m_cur_a_move_amplitude == 0) {
                if (m_debug && m_real_running) {
                    LOG_DEBUG << "WALKING: Gait was stopped";
                }
                m_real_running = false;
            } else {
                m_x_move_amplitude = 0;
                m_y_move_amplitude = 0;
                m_a_move_amplitude = 0;
            }
        }
    } else if (m_time >= (m_cur_phase_time1 - TIME_UNIT / 2) && m_time < (m_cur_phase_time1 + TIME_UNIT / 2)) {
        update_param_move();
        m_phase = PHASE1;
    } else if (m_time >= (m_cur_phase_time2 - TIME_UNIT / 2) && m_time < (m_cur_phase_time2 + TIME_UNIT / 2)) {
        update_param_time();
        m_time = m_cur_phase_time2;
        m_phase = PHASE2;
        if (!m_ctrl_running) {
            if (m_cur_x_move_amplitude == 0 && m_cur_y_move_amplitude == 0 && m_cur_a_move_amplitude == 0) {
                m_real_running = false;
            } else {
                m_x_move_amplitude = 0;
                m_y_move_amplitude = 0;
                m_a_move_amplitude = 0;
            }
        }
    } else if (m_time >= (m_cur_phase_time3 - TIME_UNIT / 2) && m_time < (m_cur_phase_time3 + TIME_UNIT / 2)) {
        update_param_move();
        m_phase = PHASE3;
    }
    update_param_balance();

    // Compute endpoints
    x_swap = wsin(m_time, m_cur_x_swap_period_time, m_cur_x_swap_phase_shift, m_cur_x_swap_amplitude, m_cur_x_swap_amplitude_shift);
    y_swap = wsin(m_time, m_cur_y_swap_period_time, m_cur_y_swap_phase_shift, m_cur_y_swap_amplitude, m_cur_y_swap_amplitude_shift);
    z_swap = wsin(m_time, m_cur_z_swap_period_time, m_cur_z_swap_phase_shift, m_cur_z_swap_amplitude, m_cur_z_swap_amplitude_shift);
    a_swap = 0;
    b_swap = 0;
    c_swap = 0;

    if (m_time <= m_cur_ssp_time_start_l) {
        x_move_l = wsin(m_cur_ssp_time_start_l, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_l, m_cur_x_move_amplitude,
                        m_cur_x_move_amplitude_shift);
        y_move_l = wsin(m_cur_ssp_time_start_l, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_l, m_cur_y_move_amplitude,
                        m_cur_y_move_amplitude_shift);
        z_move_l = wsin(m_cur_ssp_time_start_l, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_l, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_l = wsin(m_cur_ssp_time_start_l, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_l, m_cur_a_move_amplitude,
                        m_cur_a_move_amplitude_shift);
        x_move_r = wsin(m_cur_ssp_time_start_l, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_l, -m_cur_x_move_amplitude,
                        -m_cur_x_move_amplitude_shift);
        y_move_r = wsin(m_cur_ssp_time_start_l, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_l, -m_cur_y_move_amplitude,
                        -m_cur_y_move_amplitude_shift);
        z_move_r = wsin(m_cur_ssp_time_start_r, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_r, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_r = wsin(m_cur_ssp_time_start_l, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_l, -m_cur_a_move_amplitude,
                        -m_cur_a_move_amplitude_shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
        m_left_start = true;
    } else if (m_time <= m_cur_ssp_time_end_l) {
        x_move_l = wsin(m_time, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_l, m_cur_x_move_amplitude,
                        m_cur_x_move_amplitude_shift);
        y_move_l = wsin(m_time, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_l, m_cur_y_move_amplitude,
                        m_cur_y_move_amplitude_shift);
        z_move_l = wsin(m_time, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_l, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_l = wsin(m_time, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_l, m_cur_a_move_amplitude,
                        m_cur_a_move_amplitude_shift);
        x_move_r = wsin(m_time, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_l, -m_cur_x_move_amplitude,
                        -m_cur_x_move_amplitude_shift);
        y_move_r = wsin(m_time, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_l, -m_cur_y_move_amplitude,
                        -m_cur_y_move_amplitude_shift);
        z_move_r = wsin(m_cur_ssp_time_start_r, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_r, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_r = wsin(m_time, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_l, -m_cur_a_move_amplitude,
                        -m_cur_a_move_amplitude_shift);
        pelvis_offset_l = wsin(m_time, m_cur_z_move_period_time,
                               m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_l,
                               m_cur_pelvis_swing / 2, m_cur_pelvis_swing / 2);
        pelvis_offset_r = wsin(m_time, m_cur_z_move_period_time,
                               m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_l,
                               -m_cur_pelvis_offset / 2, -m_cur_pelvis_offset / 2);
        m_left_end = true;
    } else if (m_time <= m_cur_ssp_time_start_r) {
        x_move_l = wsin(m_cur_ssp_time_end_l, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_l, m_cur_x_move_amplitude,
                        m_cur_x_move_amplitude_shift);
        y_move_l = wsin(m_cur_ssp_time_end_l, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_l, m_cur_y_move_amplitude,
                        m_cur_y_move_amplitude_shift);
        z_move_l = wsin(m_cur_ssp_time_end_l, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_l, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_l = wsin(m_cur_ssp_time_end_l, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_l, m_cur_a_move_amplitude,
                        m_cur_a_move_amplitude_shift);
        x_move_r = wsin(m_cur_ssp_time_end_l, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_l, -m_cur_x_move_amplitude,
                        -m_cur_x_move_amplitude_shift);
        y_move_r = wsin(m_cur_ssp_time_end_l, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_l, -m_cur_y_move_amplitude,
                        -m_cur_y_move_amplitude_shift);
        z_move_r = wsin(m_cur_ssp_time_start_r, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_r, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_r = wsin(m_cur_ssp_time_end_l, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_l, -m_cur_a_move_amplitude,
                        -m_cur_a_move_amplitude_shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
        m_right_start = true;
    } else if (m_time <= m_cur_ssp_time_end_r) {
        x_move_l = wsin(m_time, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_r + pi,
                        m_cur_x_move_amplitude, m_cur_x_move_amplitude_shift);
        y_move_l = wsin(m_time, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_r + pi,
                        m_cur_y_move_amplitude, m_cur_y_move_amplitude_shift);
        z_move_l = wsin(m_cur_ssp_time_end_l, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_l, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_l = wsin(m_time, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_r + pi,
                        m_cur_a_move_amplitude, m_cur_a_move_amplitude_shift);
        x_move_r = wsin(m_time, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_r + pi,
                        -m_cur_x_move_amplitude, -m_cur_x_move_amplitude_shift);
        y_move_r = wsin(m_time, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_r + pi,
                        -m_cur_y_move_amplitude, -m_cur_y_move_amplitude_shift);
        z_move_r = wsin(m_time, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_r, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_r = wsin(m_time, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_r + pi,
                        -m_cur_a_move_amplitude, -m_cur_a_move_amplitude_shift);
        pelvis_offset_l = wsin(m_time, m_cur_z_move_period_time,
                               m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_r,
                               m_cur_pelvis_offset / 2, m_cur_pelvis_offset / 2);
        pelvis_offset_r = wsin(m_time, m_cur_z_move_period_time,
                               m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_r,
                               -m_cur_pelvis_swing / 2, -m_cur_pelvis_swing / 2);
        m_right_end = true;
    } else {
        x_move_l = wsin(m_cur_ssp_time_end_r, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_r + pi,
                        m_cur_x_move_amplitude, m_cur_x_move_amplitude_shift);
        y_move_l = wsin(m_cur_ssp_time_end_r, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_r + pi,
                        m_cur_y_move_amplitude, m_cur_y_move_amplitude_shift);
        z_move_l = wsin(m_cur_ssp_time_end_l, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_l, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_l = wsin(m_cur_ssp_time_end_r, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_r + pi,
                        m_cur_a_move_amplitude, m_cur_a_move_amplitude_shift);
        x_move_r = wsin(m_cur_ssp_time_end_r, m_cur_x_move_period_time,
                        m_cur_x_move_phase_shift + two_pi / m_cur_x_move_period_time * m_cur_ssp_time_start_r + pi,
                        -m_cur_x_move_amplitude, -m_cur_x_move_amplitude_shift);
        y_move_r = wsin(m_cur_ssp_time_end_r, m_cur_y_move_period_time,
                        m_cur_y_move_phase_shift + two_pi / m_cur_y_move_period_time * m_cur_ssp_time_start_r + pi,
                        -m_cur_y_move_amplitude, -m_cur_y_move_amplitude_shift);
        z_move_r = wsin(m_cur_ssp_time_end_r, m_cur_z_move_period_time,
                        m_cur_z_move_phase_shift + two_pi / m_cur_z_move_period_time * m_cur_ssp_time_start_r, m_cur_z_move_amplitude,
                        m_cur_z_move_amplitude_shift);
        c_move_r = wsin(m_cur_ssp_time_end_r, m_cur_a_move_period_time,
                        m_cur_a_move_phase_shift + two_pi / m_cur_a_move_period_time * m_cur_ssp_time_start_r + pi,
                        -m_cur_a_move_amplitude, -m_cur_a_move_amplitude_shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }

    a_move_l = 0;
    b_move_l = 0;
    a_move_r = 0;
    b_move_r = 0;

    ep[0] = x_swap + x_move_r + m_cur_x_offset;
    ep[1] = y_swap + y_move_r - m_cur_y_offset / 2;
    ep[2] = z_swap + z_move_r + m_cur_z_offset;
    ep[3] = a_swap + a_move_r - m_cur_r_offset / 2;
    ep[4] = b_swap + b_move_r + m_cur_p_offset;
    ep[5] = c_swap + c_move_r - m_cur_a_offset / 2;
    ep[6] = x_swap + x_move_l + m_cur_x_offset;
    ep[7] = y_swap + y_move_l + m_cur_y_offset / 2;
    ep[8] = z_swap + z_move_l + m_cur_z_offset;
    ep[9] = a_swap + a_move_l + m_cur_r_offset / 2;
    ep[10] = b_swap + b_move_l + m_cur_p_offset;
    ep[11] = c_swap + c_move_l + m_cur_a_offset / 2;

    if(m_left_end && m_left_start) {
        if(m_x_move_amplitude != 0 || m_y_move_amplitude != 0) {
            m_odo_x += m_left_odo_x;
            m_odo_y += m_left_odo_y;
        }
        m_odo_theta += m_left_odo_theta;
        m_left_end = false;
        m_left_start = false;
        m_right_end = false;
    }
    else if(m_right_end && m_right_start) {
        if(m_x_move_amplitude != 0 || m_y_move_amplitude != 0) {
            m_odo_x += m_right_odo_x;
            m_odo_y += m_right_odo_y;
        }
        m_odo_theta += m_right_odo_theta;
        m_right_end = false;
        m_right_start = false;
        m_left_end = false;
    } else {
        if(m_x_move_amplitude > 0) {
            m_left_odo_x = -m_odo_x_factor * ep[6];
        }
        else {
            m_left_odo_x = m_odo_x_factor * ep[6];
        }
        m_left_odo_y = -m_odo_y_factor * ep[7];
        m_left_odo_theta = -m_odo_a_factor * ep[11];

        if(m_x_move_amplitude > 0) {
            m_right_odo_x = -m_odo_x_factor * ep[0];
        }
        else {
            m_right_odo_x = m_odo_x_factor * ep[0];
        }
        m_right_odo_y = -m_odo_y_factor * ep[1];
        m_right_odo_theta = -m_odo_a_factor * ep[5];
    }

    pose_2D_t odo_offset = get_odo_offset();
//    std::cout << "Odo: " << odo_offset.x() << ' ' << odo_offset.Y() << ' ' << odo_offset.Theta() << std::endl;
    m_odometry_collector.odo_translate(odo_offset);

    // Compute body swing
    if (m_time <= m_cur_ssp_time_end_l) {
        m_cur_body_swing_y = -ep[7];
        m_cur_body_swing_z = ep[8];
    } else {
        m_cur_body_swing_y = -ep[1];
        m_cur_body_swing_z = ep[2];
    }
    m_cur_body_swing_z -= kinematics_t::LEG_LENGTH;

    // Compute arm swing
    if (m_cur_x_move_amplitude == 0) {
        angle[12] = 0; // Right
        angle[13] = 0; // Left
    } else {
        angle[12] = wsin(m_time, m_cur_period_time, pi * 1.5f, -m_cur_x_move_amplitude * m_cur_arm_swing_gain, 0);
        angle[13] = wsin(m_time, m_cur_period_time, pi * 1.5f, m_cur_x_move_amplitude * m_cur_arm_swing_gain, 0);
    }

    if (m_real_running) {
        m_time += TIME_UNIT;
        if (m_time >= m_cur_period_time)
            m_time = 0;
    }

    // Compute angles
    if ((kinematics_t::compute_leg_inverse_kinematics(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == 1)
        && (kinematics_t::compute_leg_inverse_kinematics(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == 1)) {
        for (int i = 0; i < 12; i++)
            angle[i] = degrees(angle[i]);
    } else {
        return; // Do not use angle;
    }

    // Compute motor value
    for (int i = 0; i < 14; i++) {
        offset = (float) dir[i] * angle[i] * MX28_t::RATIO_DEGREES2VALUE;
        if (i == 1) // R_HIP_ROLL
            offset += (float) dir[i] * (pelvis_offset_r /** cos(ep[5])*/ - m_hip_pitch_offset * sin(ep[5]) * MX28_t::RATIO_DEGREES2VALUE);
        else if (i == 7) // L_HIP_ROLL
            offset += (float) dir[i] * (pelvis_offset_l /** cos(ep[11])*/ - m_hip_pitch_offset * sin(ep[11]) * MX28_t::RATIO_DEGREES2VALUE);
        else if (i == 2) // R_HIP_PITCH
            offset += (float) dir[i] * (/*-pelvis_offset_r * sin(ep[5])*/ - m_hip_pitch_offset * cos(ep[5]) ) * MX28_t::RATIO_DEGREES2VALUE;
        else if (i == 8) // L_HIP_PITCH
            offset += (float) dir[i] * (/*-pelvis_offset_l * sin(ep[11])*/ - m_hip_pitch_offset * cos(ep[11])) * MX28_t::RATIO_DEGREES2VALUE;

        outValue[i] = MX28_t::angle_2_value(initAngle[i]) + (int) offset;
    }

//    // Compute motor value
//    for (int i = 0; i < 14; i++) {
//        offset = (float) dir[i] * angle[i] * MX28_t::RATIO_DEGREES2VALUE;
//        if (i == 1) // R_HIP_ROLL
//            offset += (float) dir[i] * pelvis_offset_r;
//        else if (i == 7) // L_HIP_ROLL
//            offset += (float) dir[i] * pelvis_offset_l;
//        else if (i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
//            offset -= (float) dir[i] * m_hip_pitch_offset * MX28_t::RATIO_DEGREES2VALUE;
//
//        outValue[i] = MX28_t::angle_2_value(initAngle[i]) + (int) offset;
//    }

    // adjust balance offset
    if (m_balance_enable) {
        float rlGyroErr = motion_status_t::RL_GYRO;
        float fbGyroErr = motion_status_t::FB_GYRO;
        outValue[1] += (int) (dir[1] * rlGyroErr * m_balance_hip_roll_gain * 4); // R_HIP_ROLL
        outValue[7] += (int) (dir[7] * rlGyroErr * m_balance_hip_roll_gain * 4); // L_HIP_ROLL

        outValue[3] -= (int) (dir[3] * fbGyroErr * m_balance_knee_gain * 4); // R_KNEE
        outValue[9] -= (int) (dir[9] * fbGyroErr * m_balance_knee_gain * 4); // L_KNEE

        outValue[4] -= (int) (dir[4] * fbGyroErr * m_balance_ankle_pitch_gain * 4); // R_ANKLE_PITCH
        outValue[10] -= (int) (dir[10] * fbGyroErr * m_balance_ankle_pitch_gain * 4); // L_ANKLE_PITCH

        outValue[5] -= (int) (dir[5] * rlGyroErr * m_balance_ankle_roll_gain * 4); // R_ANKLE_ROLL
        outValue[11] -= (int) (dir[11] * rlGyroErr * m_balance_ankle_roll_gain * 4); // L_ANKLE_ROLL
    }

    joint.set_value(joint_data_t::ID_R_HIP_YAW, outValue[0]);
    joint.set_value(joint_data_t::ID_R_HIP_ROLL, outValue[1]);
    joint.set_value(joint_data_t::ID_R_HIP_PITCH, outValue[2]);
    joint.set_value(joint_data_t::ID_R_KNEE, outValue[3]);
    joint.set_value(joint_data_t::ID_R_ANKLE_PITCH, outValue[4]);
    joint.set_value(joint_data_t::ID_R_ANKLE_ROLL, outValue[5]);
    joint.set_value(joint_data_t::ID_L_HIP_YAW, outValue[6]);
    joint.set_value(joint_data_t::ID_L_HIP_ROLL, outValue[7]);
    joint.set_value(joint_data_t::ID_L_HIP_PITCH, outValue[8]);
    joint.set_value(joint_data_t::ID_L_KNEE, outValue[9]);
    joint.set_value(joint_data_t::ID_L_ANKLE_PITCH, outValue[10]);
    joint.set_value(joint_data_t::ID_L_ANKLE_ROLL, outValue[11]);
    joint.set_value(joint_data_t::ID_R_SHOULDER_PITCH, outValue[12]);
    joint.set_value(joint_data_t::ID_L_SHOULDER_PITCH, outValue[13]);
    joint.set_angle(joint_data_t::ID_HEAD_PAN, m_a_move_amplitude);

    for (int id = joint_data_t::ID_R_HIP_YAW; id <= joint_data_t::ID_L_ANKLE_ROLL; id++) {
        joint.set_p_gain(id, m_p_gain);
        joint.set_i_gain(id, m_i_gain);
        joint.set_d_gain(id, m_d_gain);
    }
}

pose_2D_t walking_t::get_odo_offset() {
    pose_2D_t offset(m_odo_x, m_odo_y, m_odo_theta);
    m_odo_x = 0;
    m_odo_y = 0;
    m_odo_theta = 0;
    return offset;
}

pose_2D_t walking_t::get_odo() {
    return m_odometry_collector.get_pose();
}

void walking_t::reset_odo(const pose_2D_t& pose) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: Odometry has been reset";
    }
    m_odometry_collector.reset();
}

void walking_t::set_odo(const pose_2D_t& pose) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: odo_x = " << pose.x()
                  << ", odo_y = " << pose.y()
                  << ", odo_theta = " << pose.theta();
    }
    m_odometry_collector.set_pose(pose);
}

walking_t* walking_t::get_instance() {
    static walking_t walking;
    return &walking;
}

float walking_t::get_x_offset() const {
    return m_x_offset;
}

void walking_t::set_x_offset(float x_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: x_offset" << x_offset;
    }
    walking_t::m_x_offset = x_offset;
}

float walking_t::get_y_offset() const {
    return m_y_offset;
}

void walking_t::set_y_offset(float y_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: y_offset" << y_offset;
    }
    walking_t::m_y_offset = y_offset;
}

float walking_t::get_z_offset() const {
    return m_z_offset;
}

void walking_t::set_z_offset(float z_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: z_offset" << z_offset;
    }
    walking_t::m_z_offset = z_offset;
}

float walking_t::get_yaw_offset() const {
    return m_a_offset;
}

void walking_t::set_yaw_offset(float a_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: yaw_offset" << a_offset;
    }
    walking_t::m_a_offset = a_offset;
}

float walking_t::get_pitch_offset() const {
    return m_p_offset;
}

void walking_t::set_pitch_offset(float p_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: pitch_offset" << p_offset;
    }
    walking_t::m_p_offset = p_offset;
}

float walking_t::get_roll_offset() const {
    return m_r_offset;
}

void walking_t::set_roll_offset(float r_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: roll_offset" << r_offset;
    }
    walking_t::m_r_offset = r_offset;
}

float walking_t::get_period_time() const {
    return m_period_time;
}

void walking_t::set_period_time(float period_time) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: period_time" << period_time;
    }
    walking_t::m_period_time = period_time;
}

float walking_t::get_DSP_ratio() const {
    return m_dsp_ratio;
}

void walking_t::set_DSP_ratio(float dsp_ratio) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: dsp_ratio" << dsp_ratio;
    }
    walking_t::m_dsp_ratio = dsp_ratio;
}

float walking_t::get_step_FB_ratio() const {
    return m_step_fb_ratio;
}

void walking_t::set_step_FB_ratio(float step_fb_ratio) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: step_fb_ration" << step_fb_ratio;
    }
    walking_t::m_step_fb_ratio = step_fb_ratio;
}

float walking_t::get_x_move_amplitude() const {
    return m_x_move_amplitude;
}

void walking_t::set_x_move_amplitude(float x_move_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: x_move_amplitude = " << x_move_amplitude;
    }
    walking_t::m_x_move_amplitude = x_move_amplitude;
}

float walking_t::get_y_move_amplitude() const {
    return m_y_move_amplitude;
}

void walking_t::set_y_move_amplitude(float y_move_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: y_move_amplitude = " << y_move_amplitude;
    }
    walking_t::m_y_move_amplitude = y_move_amplitude;
}

float walking_t::get_z_move_amplitude() const {
    return m_z_move_amplitude;
}

void walking_t::set_z_move_amplitude(float z_move_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: z_move_amplitude = " << z_move_amplitude;
    }
    walking_t::m_z_move_amplitude = z_move_amplitude;
}

float walking_t::get_a_move_amplitude() const {
    return m_a_move_amplitude;
}

void walking_t::set_a_move_amplitude(float a_move_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: a_move_amplitude = " << a_move_amplitude;
    }
    walking_t::m_a_move_amplitude = a_move_amplitude;
}

bool walking_t::get_a_move_aim_on() const {
    return m_move_aim_on;
}

void walking_t::set_move_aim_on(bool move_aim_on) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: move_aim_on = " << std::boolalpha << move_aim_on;
    }
    walking_t::m_move_aim_on = move_aim_on;
}

bool walking_t::get_balance_enable() const {
    return m_balance_enable;
}

void walking_t::set_balance_enable(bool balance_enable) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_enabled = " << std::boolalpha << balance_enable;
    }
    walking_t::m_balance_enable = balance_enable;
}

float walking_t::get_balance_knee_gain() const {
    return m_balance_knee_gain;
}

void walking_t::set_balance_knee_gain(float balance_knee_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_knee_gain = " << balance_knee_gain;
    }
    walking_t::m_balance_knee_gain = balance_knee_gain;
}

float walking_t::get_balance_ankle_pitch_gain() const {
    return m_balance_ankle_pitch_gain;
}

void walking_t::set_balance_ankle_pitch_gain(float balance_ankle_pitch_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_ankle_pitch_gain = " << balance_ankle_pitch_gain;
    }
    walking_t::m_balance_ankle_pitch_gain = balance_ankle_pitch_gain;
}

float walking_t::get_balance_hip_roll_gain() const {
    return m_balance_hip_roll_gain;
}

void walking_t::set_balance_hip_roll_gain(float balance_hip_roll_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_hip_roll_gain = " << balance_hip_roll_gain;
    }
    walking_t::m_balance_hip_roll_gain = balance_hip_roll_gain;
}

float walking_t::get_balance_ankle_roll_gain() const {
    return m_balance_ankle_roll_gain;
}

void walking_t::set_balance_ankle_roll_gain(float balance_ankle_roll_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_ankle_roll_gain = " << balance_ankle_roll_gain;
    }
    walking_t::m_balance_ankle_roll_gain = balance_ankle_roll_gain;
}

float walking_t::get_y_swap_amplitude() const {
    return m_y_swap_amplitude;
}

void walking_t::set_y_swap_amplitude(float y_swap_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: y_swap_amplitude = " << y_swap_amplitude;
    }
    walking_t::m_y_swap_amplitude = y_swap_amplitude;
}

float walking_t::get_z_swap_amplitude() const {
    return m_z_swap_amplitude;
}

void walking_t::set_z_swap_amplitude(float z_swap_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: z_swap_amplitude = " << z_swap_amplitude;
    }
    walking_t::m_z_swap_amplitude = z_swap_amplitude;
}

float walking_t::get_arm_swing_gain() const {
    return m_arm_swing_gain;
}

void walking_t::set_arm_swing_gain(float arm_swing_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: arm_swing_gain = " << arm_swing_gain;
    }
    walking_t::m_arm_swing_gain = arm_swing_gain;
}

float walking_t::get_pelvis_offset() const {
    return m_pelvis_offset;
}

void walking_t::set_pelvis_offset(float pelvis_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: pelvis_offset = " << pelvis_offset;
    }
    walking_t::m_pelvis_offset = pelvis_offset;
}

float walking_t::get_hip_pitch_offset() const {
    return m_hip_pitch_offset;
}

void walking_t::set_hip_pitch_offset(float hip_pitch_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: hip_pitch_offset = " << hip_pitch_offset;
    }
    walking_t::m_hip_pitch_offset = hip_pitch_offset;
}

int walking_t::get_p_gain() const {
    return m_p_gain;
}

void walking_t::set_p_gain(int p_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: p_gain = " << p_gain;
    }
    walking_t::m_p_gain = p_gain;
}

int walking_t::get_i_gain() const {
    return m_i_gain;
}

void walking_t::set_i_gain(int i_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: i_gain = " << i_gain;
    }
    walking_t::m_i_gain = i_gain;
}

int walking_t::get_d_gain() const {
    return m_d_gain;
}

void walking_t::set_d_gain(int d_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: d_gain = " << d_gain;
    }
    walking_t::m_d_gain = d_gain;
}

float walking_t::get_odo_x_factor() const {
    return m_odo_x_factor;
}

void walking_t::set_odo_x_factor(float odo_x_factor) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: odo_x_factor = " << odo_x_factor;
    }
    m_odo_x_factor = odo_x_factor;
}

float walking_t::get_odo_y_factor() const {
    return m_odo_y_factor;
}

void walking_t::set_odo_y_factor(float odo_y_factor) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: odo_y_factor = " << odo_y_factor;
    }
    m_odo_y_factor = odo_y_factor;
}

float walking_t::get_odo_a_factor() const {
    return m_odo_a_factor;
}

void walking_t::set_odo_a_factor(float odo_a_factor) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: odo_a_factor = " << odo_a_factor;
    }
    m_odo_a_factor = odo_a_factor;
}

bool walking_t::is_debug_enabled() const {
    return m_debug;
}

void walking_t::enable_debug(bool debug) {
    m_debug = debug;
}
