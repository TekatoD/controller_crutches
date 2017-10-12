/*
 *   Walking.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <iostream>
#include <math/AngleTools.h>
#include <log/Logger.h>
#include "MX28.h"
#include "motion/MotionStatus.h"
#include "motion/Kinematics.h"
#include "motion/modules/Walking.h"

using namespace Robot;

Walking::Walking() {
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

    m_p_gain = JointData::P_GAIN_DEFAULT;
    m_i_gain = JointData::I_GAIN_DEFAULT;
    m_d_gain = JointData::D_GAIN_DEFAULT;

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

    m_Joint.SetAngle(JointData::ID_R_SHOULDER_PITCH, -48.345f);
    m_Joint.SetAngle(JointData::ID_L_SHOULDER_PITCH, 41.313f);
    m_Joint.SetAngle(JointData::ID_R_SHOULDER_ROLL, -17.873f);
    m_Joint.SetAngle(JointData::ID_L_SHOULDER_ROLL, 17.580f);
    m_Joint.SetAngle(JointData::ID_R_ELBOW, 29.300f);
    m_Joint.SetAngle(JointData::ID_L_ELBOW, -29.593f);

    m_Joint.SetAngle(JointData::ID_HEAD_TILT, Kinematics::EYE_TILT_OFFSET_ANGLE);

    m_Joint.SetPGain(JointData::ID_R_SHOULDER_PITCH, 8);
    m_Joint.SetPGain(JointData::ID_L_SHOULDER_PITCH, 8);
    m_Joint.SetPGain(JointData::ID_R_SHOULDER_ROLL, 8);
    m_Joint.SetPGain(JointData::ID_L_SHOULDER_ROLL, 8);
    m_Joint.SetPGain(JointData::ID_R_ELBOW, 8);
    m_Joint.SetPGain(JointData::ID_L_ELBOW, 8);
}


void Walking::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, WALKING_SECTION);
}


void Walking::LoadINISettings(minIni* ini, const std::string& section) {
    m_odo_x_factor = ini->getf(section, "odo_x_factor", m_odo_x_factor);
    m_odo_y_factor = ini->getf(section, "odo_y_factor", m_odo_y_factor);
    m_odo_a_factor = ini->getf(section, "odo_a_factor", m_odo_a_factor);
    m_x_offset = ini->getf(section, "x_offset", m_x_offset);
    m_y_offset = ini->getf(section, "y_offset", m_y_offset);
    m_z_offset = ini->getf(section, "z_offset", m_z_offset);
    m_r_offset = ini->getf(section, "roll_offset", m_r_offset);
    m_p_offset = ini->getf(section, "pitch_offset", m_p_offset);
    m_a_offset = ini->getf(section, "yaw_offset", m_a_offset);
    m_hip_pitch_offset = ini->getf(section, "hip_pitch_offset", m_hip_pitch_offset);
    m_period_time = ini->getf(section, "period_time", m_period_time);
    m_dsp_ratio = ini->getf(section, "dsp_ratio", m_dsp_ratio);
    m_step_fb_ratio = ini->getf(section, "step_forward_back_ratio", m_step_fb_ratio);
    m_z_move_amplitude = ini->getf(section, "foot_height", m_z_move_amplitude);
    m_y_swap_amplitude = ini->getf(section, "swing_right_left", m_y_swap_amplitude);
    m_z_swap_amplitude = ini->getf(section, "swing_top_down", m_z_swap_amplitude);
    m_pelvis_offset = ini->getf(section, "pelvis_offset", m_pelvis_offset);
    m_arm_swing_gain = ini->getf(section, "arm_swing_gain", m_arm_swing_gain );
    m_balance_knee_gain = ini->getf(section, "balance_knee_gain", m_balance_knee_gain );
    m_balance_ankle_pitch_gain = ini->getf(section, "balance_ankle_pitch_gain", m_balance_ankle_pitch_gain);
    m_balance_hip_roll_gain = ini->getf(section, "balance_hip_roll_gain", m_balance_hip_roll_gain);
    m_balance_ankle_roll_gain = ini->getf(section, "balance_ankle_roll_gain", m_balance_ankle_roll_gain);

    m_p_gain = ini->geti(section, "p_gain", m_p_gain);
    m_i_gain = ini->geti(section, "i_gain", m_i_gain);
    m_d_gain = ini->geti(section, "d_gain", m_d_gain);
}


void Walking::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, WALKING_SECTION);
}


void Walking::SaveINISettings(minIni* ini, const std::string& section) {
    ini->put(section, "x_offset", m_x_offset);
    ini->put(section, "y_offset", m_y_offset);
    ini->put(section, "z_offset", m_z_offset);
    ini->put(section, "roll_offset", m_r_offset);
    ini->put(section, "pitch_offset", m_p_offset);
    ini->put(section, "yaw_offset", m_a_offset);
    ini->put(section, "hip_pitch_offset", m_hip_pitch_offset);
    ini->put(section, "period_time", m_period_time);
    ini->put(section, "dsp_ratio", m_dsp_ratio);
    ini->put(section, "step_forward_back_ratio", m_step_fb_ratio);
    ini->put(section, "foot_height", m_z_move_amplitude);
    ini->put(section, "swing_right_left", m_y_swap_amplitude);
    ini->put(section, "swing_top_down", m_z_swap_amplitude);
    ini->put(section, "pelvis_offset", m_pelvis_offset);
    ini->put(section, "arm_swing_gain", m_arm_swing_gain);
    ini->put(section, "balance_knee_gain", m_balance_knee_gain);
    ini->put(section, "balance_ankle_pitch_gain", m_balance_ankle_pitch_gain);
    ini->put(section, "balance_hip_roll_gain", m_balance_hip_roll_gain);
    ini->put(section, "balance_ankle_roll_gain", m_balance_ankle_roll_gain);

    ini->put(section, "p_gain", m_p_gain);
    ini->put(section, "i_gain", m_i_gain);
    ini->put(section, "d_gain", m_d_gain);
}


void Walking::UpdateParamTime() {
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

    m_cur_pelvis_offset = m_pelvis_offset * MX28::RATIO_DEGREES2VALUE;
    m_cur_pelvis_swing = m_cur_pelvis_offset * 0.35f;
    m_cur_arm_swing_gain = m_arm_swing_gain;
}


void Walking::UpdateParamMove() {
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


void Walking::UpdateParamBalance() {
    if (m_debug) {
        LOG_TRACE << "WALKING: Updating balance parameters";
    }
    m_cur_x_offset = m_x_offset;
    m_cur_y_offset = m_y_offset;
    m_cur_z_offset = m_z_offset;
    m_cur_r_offset = radians(m_r_offset);
    m_cur_p_offset = radians(m_p_offset);
    m_cur_a_offset = radians(m_a_offset);
    m_cur_hip_pitch_offset = m_hip_pitch_offset * MX28::RATIO_DEGREES2VALUE;
}


void Walking::Initialize() {
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
    UpdateParamTime();
    UpdateParamMove();

    Process();
}


void Walking::Start() {
    if (m_debug && !m_ctrl_running) {
        LOG_DEBUG << "WALKING: Gait was started";
    }
    m_ctrl_running = true;
    m_real_running = true;
}


void Walking::Stop() {
    if (m_debug) {
        LOG_DEBUG << "WALKING: Stop signal was received";
    }
    m_ctrl_running = false;
}


bool Walking::IsRunning() {
    return m_real_running;
}


void Walking::Process() {
    float x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
    float x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
    float x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
    float pelvis_offset_r, pelvis_offset_l;
    float angle[14], ep[12];
    float offset;
    constexpr float TIME_UNIT = MotionModule::TIME_UNIT;
    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    constexpr int dir[14] = {-1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1};
    constexpr float initAngle[14] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -48.345f, 41.313f};
    int outValue[14];

    // Update walk parameters
    if (m_time == 0) {
        UpdateParamTime();
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
        UpdateParamMove();
        m_phase = PHASE1;
    } else if (m_time >= (m_cur_phase_time2 - TIME_UNIT / 2) && m_time < (m_cur_phase_time2 + TIME_UNIT / 2)) {
        UpdateParamTime();
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
        UpdateParamMove();
        m_phase = PHASE3;
    }
    UpdateParamBalance();

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

    Pose2D odo_offset = GetOdoOffset();
//    std::cout << "Odo: " << odo_offset.X() << ' ' << odo_offset.Y() << ' ' << odo_offset.Theta() << std::endl;
    m_odometry_collector.odoTranslate(odo_offset);

    // Compute body swing
    if (m_time <= m_cur_ssp_time_end_l) {
        m_cur_body_swing_y = -ep[7];
        m_cur_body_swing_z = ep[8];
    } else {
        m_cur_body_swing_y = -ep[1];
        m_cur_body_swing_z = ep[2];
    }
    m_cur_body_swing_z -= Kinematics::LEG_LENGTH;

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
    if ((Kinematics::ComputeLegInverseKinematics(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == 1)
        && (Kinematics::ComputeLegInverseKinematics(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == 1)) {
        for (int i = 0; i < 12; i++)
            angle[i] = degrees(angle[i]);
    } else {
        return; // Do not use angle;
    }

    // Compute motor value
    for (int i = 0; i < 14; i++) {
        offset = (float) dir[i] * angle[i] * MX28::RATIO_DEGREES2VALUE;
        if (i == 1) // R_HIP_ROLL
            offset += (float) dir[i] * (pelvis_offset_r /** cos(ep[5])*/ - m_hip_pitch_offset * sin(ep[5]) * MX28::RATIO_DEGREES2VALUE);
        else if (i == 7) // L_HIP_ROLL
            offset += (float) dir[i] * (pelvis_offset_l /** cos(ep[11])*/ - m_hip_pitch_offset * sin(ep[11]) * MX28::RATIO_DEGREES2VALUE);
        else if (i == 2) // R_HIP_PITCH
            offset += (float) dir[i] * (/*-pelvis_offset_r * sin(ep[5])*/ - m_hip_pitch_offset * cos(ep[5]) ) * MX28::RATIO_DEGREES2VALUE;
        else if (i == 8) // L_HIP_PITCH
            offset += (float) dir[i] * (/*-pelvis_offset_l * sin(ep[11])*/ - m_hip_pitch_offset * cos(ep[11])) * MX28::RATIO_DEGREES2VALUE;

        outValue[i] = MX28::Angle2Value(initAngle[i]) + (int) offset;
    }

//    // Compute motor value
//    for (int i = 0; i < 14; i++) {
//        offset = (float) dir[i] * angle[i] * MX28::RATIO_DEGREES2VALUE;
//        if (i == 1) // R_HIP_ROLL
//            offset += (float) dir[i] * pelvis_offset_r;
//        else if (i == 7) // L_HIP_ROLL
//            offset += (float) dir[i] * pelvis_offset_l;
//        else if (i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
//            offset -= (float) dir[i] * m_hip_pitch_offset * MX28::RATIO_DEGREES2VALUE;
//
//        outValue[i] = MX28::Angle2Value(initAngle[i]) + (int) offset;
//    }

    // adjust balance offset
    if (m_balance_enable) {
        float rlGyroErr = MotionStatus::RL_GYRO;
        float fbGyroErr = MotionStatus::FB_GYRO;
        outValue[1] += (int) (dir[1] * rlGyroErr * m_balance_hip_roll_gain * 4); // R_HIP_ROLL
        outValue[7] += (int) (dir[7] * rlGyroErr * m_balance_hip_roll_gain * 4); // L_HIP_ROLL

        outValue[3] -= (int) (dir[3] * fbGyroErr * m_balance_knee_gain * 4); // R_KNEE
        outValue[9] -= (int) (dir[9] * fbGyroErr * m_balance_knee_gain * 4); // L_KNEE

        outValue[4] -= (int) (dir[4] * fbGyroErr * m_balance_ankle_pitch_gain * 4); // R_ANKLE_PITCH
        outValue[10] -= (int) (dir[10] * fbGyroErr * m_balance_ankle_pitch_gain * 4); // L_ANKLE_PITCH

        outValue[5] -= (int) (dir[5] * rlGyroErr * m_balance_ankle_roll_gain * 4); // R_ANKLE_ROLL
        outValue[11] -= (int) (dir[11] * rlGyroErr * m_balance_ankle_roll_gain * 4); // L_ANKLE_ROLL
    }

    m_Joint.SetValue(JointData::ID_R_HIP_YAW, outValue[0]);
    m_Joint.SetValue(JointData::ID_R_HIP_ROLL, outValue[1]);
    m_Joint.SetValue(JointData::ID_R_HIP_PITCH, outValue[2]);
    m_Joint.SetValue(JointData::ID_R_KNEE, outValue[3]);
    m_Joint.SetValue(JointData::ID_R_ANKLE_PITCH, outValue[4]);
    m_Joint.SetValue(JointData::ID_R_ANKLE_ROLL, outValue[5]);
    m_Joint.SetValue(JointData::ID_L_HIP_YAW, outValue[6]);
    m_Joint.SetValue(JointData::ID_L_HIP_ROLL, outValue[7]);
    m_Joint.SetValue(JointData::ID_L_HIP_PITCH, outValue[8]);
    m_Joint.SetValue(JointData::ID_L_KNEE, outValue[9]);
    m_Joint.SetValue(JointData::ID_L_ANKLE_PITCH, outValue[10]);
    m_Joint.SetValue(JointData::ID_L_ANKLE_ROLL, outValue[11]);
    m_Joint.SetValue(JointData::ID_R_SHOULDER_PITCH, outValue[12]);
    m_Joint.SetValue(JointData::ID_L_SHOULDER_PITCH, outValue[13]);
    m_Joint.SetAngle(JointData::ID_HEAD_PAN, m_a_move_amplitude);

    for (int id = JointData::ID_R_HIP_YAW; id <= JointData::ID_L_ANKLE_ROLL; id++) {
        m_Joint.SetPGain(id, m_p_gain);
        m_Joint.SetIGain(id, m_i_gain);
        m_Joint.SetDGain(id, m_d_gain);
    }
}

Pose2D Walking::GetOdoOffset() {
    Pose2D offset(m_odo_x, m_odo_y, m_odo_theta);
    m_odo_x = 0;
    m_odo_y = 0;
    m_odo_theta = 0;
    return offset;
}

Pose2D Walking::GetOdo() {
    return m_odometry_collector.GetPose();
}

void Walking::ResetOdo(const Pose2D& pose) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: Odometry was resseted";
    }
    m_odometry_collector.Reset();
}

void Walking::SetOdo(const Pose2D& pose) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: odo_x = " << pose.X()
                  << ", odo_y = " << pose.Y()
                  << ", odo_theta = " << pose.Theta();
    }
    m_odometry_collector.SetPose(pose);
}

Walking* Walking::GetInstance() {
    static Walking walking;
    return &walking;
}

float Walking::GetXOffset() const {
    return m_x_offset;
}

void Walking::SetXOffset(float x_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: x_offset" << x_offset;
    }
    Walking::m_x_offset = x_offset;
}

float Walking::GetYOffset() const {
    return m_y_offset;
}

void Walking::SetYOffset(float y_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: y_offset" << y_offset;
    }
    Walking::m_y_offset = y_offset;
}

float Walking::GetZOffset() const {
    return m_z_offset;
}

void Walking::SetZOffset(float z_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: z_offset" << z_offset;
    }
    Walking::m_z_offset = z_offset;
}

float Walking::GetAOffset() const {
    return m_a_offset;
}

void Walking::SetAOffset(float a_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: a_offset" << a_offset;
    }
    Walking::m_a_offset = a_offset;
}

float Walking::GetPOffset() const {
    return m_p_offset;
}

void Walking::SetPOffset(float p_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: p_offset" << p_offset;
    }
    Walking::m_p_offset = p_offset;
}

float Walking::GetROffset() const {
    return m_r_offset;
}

void Walking::SetROffset(float r_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: r_offset" << r_offset;
    }
    Walking::m_r_offset = r_offset;
}

float Walking::GetPeriodTime() const {
    return m_period_time;
}

void Walking::SetPeriodTime(float period_time) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: period_time" << period_time;
    }
    Walking::m_period_time = period_time;
}

float Walking::GetDSPRatio() const {
    return m_dsp_ratio;
}

void Walking::SetDSPRatio(float dsp_ratio) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: dsp_ratio" << dsp_ratio;
    }
    Walking::m_dsp_ratio = dsp_ratio;
}

float Walking::GetStepFBRatio() const {
    return m_step_fb_ratio;
}

void Walking::SetStepFBRatio(float step_fb_ratio) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: step_fb_ration" << step_fb_ratio;
    }
    Walking::m_step_fb_ratio = step_fb_ratio;
}

float Walking::GetXMoveAmplitude() const {
    return m_x_move_amplitude;
}

void Walking::SetXMoveAmplitude(float x_move_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: x_move_amplitude = " << x_move_amplitude;
    }
    Walking::m_x_move_amplitude = x_move_amplitude;
}

float Walking::GetYMoveAmplitude() const {
    return m_y_move_amplitude;
}

void Walking::SetYMoveAmplitude(float y_move_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: y_move_amplitude = " << y_move_amplitude;
    }
    Walking::m_y_move_amplitude = y_move_amplitude;
}

float Walking::GetZMoveAmplitude() const {
    return m_z_move_amplitude;
}

void Walking::SetZMoveAmplitude(float z_move_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: z_move_amplitude = " << z_move_amplitude;
    }
    Walking::m_z_move_amplitude = z_move_amplitude;
}

float Walking::GetAMoveAmplitude() const {
    return m_a_move_amplitude;
}

void Walking::SetAMoveAmplitude(float a_move_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: a_move_amplitude = " << a_move_amplitude;
    }
    Walking::m_a_move_amplitude = a_move_amplitude;
}

bool Walking::GetAMoveAimOn() const {
    return m_move_aim_on;
}

void Walking::SetMoveAimOn(bool move_aim_on) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: move_aim_on = " << std::boolalpha << move_aim_on;
    }
    Walking::m_move_aim_on = move_aim_on;
}

bool Walking::GetBalanceEnable() const {
    return m_balance_enable;
}

void Walking::SetBalanceEnable(bool balance_enable) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_enabled = " << std::boolalpha << balance_enable;
    }
    Walking::m_balance_enable = balance_enable;
}

float Walking::GetBalanceKneeGain() const {
    return m_balance_knee_gain;
}

void Walking::SetBalanceKneeGain(float balance_knee_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_knee_gain = " << balance_knee_gain;
    }
    Walking::m_balance_knee_gain = balance_knee_gain;
}

float Walking::GetBalanceAnklePitchGain() const {
    return m_balance_ankle_pitch_gain;
}

void Walking::SetBalanceAnklePitchGain(float balance_ankle_pitch_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_ankle_pitch_gain = " << balance_ankle_pitch_gain;
    }
    Walking::m_balance_ankle_pitch_gain = balance_ankle_pitch_gain;
}

float Walking::GetBalanceHipRollGain() const {
    return m_balance_hip_roll_gain;
}

void Walking::SetBalanceHipRollGain(float balance_hip_roll_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_hip_roll_gain = " << balance_hip_roll_gain;
    }
    Walking::m_balance_hip_roll_gain = balance_hip_roll_gain;
}

float Walking::GetBalanceAnkleRollGain() const {
    return m_balance_ankle_roll_gain;
}

void Walking::SetBalanceAnkleRollGain(float balance_ankle_roll_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: balance_ankle_roll_gain = " << balance_ankle_roll_gain;
    }
    Walking::m_balance_ankle_roll_gain = balance_ankle_roll_gain;
}

float Walking::GetYSwapAmplitude() const {
    return m_y_swap_amplitude;
}

void Walking::SetYSwapAmplitude(float y_swap_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: y_swap_amplitude = " << y_swap_amplitude;
    }
    Walking::m_y_swap_amplitude = y_swap_amplitude;
}

float Walking::GetZSwapAmplitude() const {
    return m_z_swap_amplitude;
}

void Walking::SetZSwapAmplitude(float z_swap_amplitude) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: z_swap_amplitude = " << z_swap_amplitude;
    }
    Walking::m_z_swap_amplitude = z_swap_amplitude;
}

float Walking::GetArmSwingGain() const {
    return m_arm_swing_gain;
}

void Walking::SetArmSwingGain(float arm_swing_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: arm_swing_gain = " << arm_swing_gain;
    }
    Walking::m_arm_swing_gain = arm_swing_gain;
}

float Walking::GetPelvisOffset() const {
    return m_pelvis_offset;
}

void Walking::SetPelvisOffset(float pelvis_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: pelvis_offset = " << pelvis_offset;
    }
    Walking::m_pelvis_offset = pelvis_offset;
}

float Walking::GetHipPitchOffset() const {
    return m_hip_pitch_offset;
}

void Walking::SetHipPitchOffset(float hip_pitch_offset) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: hip_pitch_offset = " << hip_pitch_offset;
    }
    Walking::m_hip_pitch_offset = hip_pitch_offset;
}

int Walking::GetPGain() const {
    return m_p_gain;
}

void Walking::SetPGain(int p_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: p_gain = " << p_gain;
    }
    Walking::m_p_gain = p_gain;
}

int Walking::GetIGain() const {
    return m_i_gain;
}

void Walking::SetIGain(int i_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: i_gain = " << i_gain;
    }
    Walking::m_i_gain = i_gain;
}

int Walking::GetDGain() const {
    return m_d_gain;
}

void Walking::SetDGain(int d_gain) {
    if (m_debug) {
        LOG_DEBUG << "WALKING: d_gain = " << d_gain;
    }
    Walking::m_d_gain = d_gain;
}

bool Walking::GetDebug() const {
    return m_debug;
}

void Walking::SetDebug(bool debug) {
    m_debug = debug;
}
