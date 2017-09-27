/*
 *   Walking.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <iostream>
#include <math/AngleTools.h>
#include "MX28.h"
#include "motion/MotionStatus.h"
#include "motion/Kinematics.h"
#include "motion/modules/Walking.h"

using namespace Robot;

Walking::Walking() {
    X_OFFSET = -10;
    Y_OFFSET = 5;
    Z_OFFSET = 20;
    R_OFFSET = 0;
    P_OFFSET = 0;
    A_OFFSET = 0;
    HIP_PITCH_OFFSET = 13.0;
    PERIOD_TIME = 600;
    DSP_RATIO = 0.1;
    STEP_FB_RATIO = 0.28;
    Z_MOVE_AMPLITUDE = 40;
    Y_SWAP_AMPLITUDE = 20.0;
    Z_SWAP_AMPLITUDE = 5;
    PELVIS_OFFSET = 3.0;
    ARM_SWING_GAIN = 1.5;
    BALANCE_KNEE_GAIN = 0.3;
    BALANCE_ANKLE_PITCH_GAIN = 0.9;
    BALANCE_HIP_ROLL_GAIN = 0.5;
    BALANCE_ANKLE_ROLL_GAIN = 1.0;

    P_GAIN = JointData::P_GAIN_DEFAULT;
    I_GAIN = JointData::I_GAIN_DEFAULT;
    D_GAIN = JointData::D_GAIN_DEFAULT;

    X_MOVE_AMPLITUDE = 0;
    Y_MOVE_AMPLITUDE = 0;
    A_MOVE_AMPLITUDE = 0;
    A_MOVE_AIM_ON = false;
    BALANCE_ENABLE = true;

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

    m_Joint.SetAngle(JointData::ID_R_SHOULDER_PITCH, -48.345);
    m_Joint.SetAngle(JointData::ID_L_SHOULDER_PITCH, 41.313);
    m_Joint.SetAngle(JointData::ID_R_SHOULDER_ROLL, -17.873);
    m_Joint.SetAngle(JointData::ID_L_SHOULDER_ROLL, 17.580);
    m_Joint.SetAngle(JointData::ID_R_ELBOW, 29.300);
    m_Joint.SetAngle(JointData::ID_L_ELBOW, -29.593);

    m_Joint.SetAngle(JointData::ID_HEAD_TILT, Kinematics::EYE_TILT_OFFSET_ANGLE);

    m_Joint.SetPGain(JointData::ID_R_SHOULDER_PITCH, 8);
    m_Joint.SetPGain(JointData::ID_L_SHOULDER_PITCH, 8);
    m_Joint.SetPGain(JointData::ID_R_SHOULDER_ROLL, 8);
    m_Joint.SetPGain(JointData::ID_L_SHOULDER_ROLL, 8);
    m_Joint.SetPGain(JointData::ID_R_ELBOW, 8);
    m_Joint.SetPGain(JointData::ID_L_ELBOW, 8);
}


Walking::~Walking() {
}


void Walking::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, WALKING_SECTION);
}


void Walking::LoadINISettings(minIni* ini, const std::string& section) {
    m_odo_x_factor = ini->getf(section, "odo_x_factor", m_odo_x_factor);
    m_odo_y_factor = ini->getf(section, "odo_y_factor", m_odo_y_factor);
    m_odo_a_factor = ini->getf(section, "odo_a_factor", m_odo_a_factor);
    X_OFFSET = ini->getf(section, "x_offset", X_OFFSET);
    Y_OFFSET = ini->getf(section, "y_offset", Y_OFFSET);
    Z_OFFSET = ini->getf(section, "z_offset", Z_OFFSET);
    R_OFFSET = ini->getf(section, "roll_offset", R_OFFSET);
    P_OFFSET = ini->getf(section, "pitch_offset", P_OFFSET);
    A_OFFSET = ini->getf(section, "yaw_offset", A_OFFSET);
    HIP_PITCH_OFFSET = ini->getf(section, "hip_pitch_offset", HIP_PITCH_OFFSET);
    PERIOD_TIME = ini->getf(section, "period_time", PERIOD_TIME);
    DSP_RATIO = ini->getf(section, "dsp_ratio", DSP_RATIO);
    STEP_FB_RATIO = ini->getf(section, "step_forward_back_ratio", STEP_FB_RATIO);
    Z_MOVE_AMPLITUDE = ini->getf(section, "foot_height", Z_MOVE_AMPLITUDE);
    Y_SWAP_AMPLITUDE = ini->getf(section, "swing_right_left", Y_SWAP_AMPLITUDE);
    Z_SWAP_AMPLITUDE = ini->getf(section, "swing_top_down", Z_SWAP_AMPLITUDE);
    PELVIS_OFFSET = ini->getf(section, "pelvis_offset", PELVIS_OFFSET);
    ARM_SWING_GAIN = ini->getf(section, "arm_swing_gain", ARM_SWING_GAIN );
    BALANCE_KNEE_GAIN = ini->getf(section, "balance_knee_gain", BALANCE_KNEE_GAIN );
    BALANCE_ANKLE_PITCH_GAIN = ini->getf(section, "balance_ankle_pitch_gain", BALANCE_ANKLE_PITCH_GAIN);
    BALANCE_HIP_ROLL_GAIN = ini->getf(section, "balance_hip_roll_gain", BALANCE_HIP_ROLL_GAIN);
    BALANCE_ANKLE_ROLL_GAIN = ini->getf(section, "balance_ankle_roll_gain", BALANCE_ANKLE_ROLL_GAIN);

    P_GAIN = ini->geti(section, "p_gain", P_GAIN);
    I_GAIN = ini->geti(section, "i_gain", I_GAIN);
    D_GAIN = ini->geti(section, "d_gain", D_GAIN);
}


void Walking::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, WALKING_SECTION);
}


void Walking::SaveINISettings(minIni* ini, const std::string& section) {
    ini->put(section, "x_offset", X_OFFSET);
    ini->put(section, "y_offset", Y_OFFSET);
    ini->put(section, "z_offset", Z_OFFSET);
    ini->put(section, "roll_offset", R_OFFSET);
    ini->put(section, "pitch_offset", P_OFFSET);
    ini->put(section, "yaw_offset", A_OFFSET);
    ini->put(section, "hip_pitch_offset", HIP_PITCH_OFFSET);
    ini->put(section, "period_time", PERIOD_TIME);
    ini->put(section, "dsp_ratio", DSP_RATIO);
    ini->put(section, "step_forward_back_ratio", STEP_FB_RATIO);
    ini->put(section, "foot_height", Z_MOVE_AMPLITUDE);
    ini->put(section, "swing_right_left", Y_SWAP_AMPLITUDE);
    ini->put(section, "swing_top_down", Z_SWAP_AMPLITUDE);
    ini->put(section, "pelvis_offset", PELVIS_OFFSET);
    ini->put(section, "arm_swing_gain", ARM_SWING_GAIN);
    ini->put(section, "balance_knee_gain", BALANCE_KNEE_GAIN);
    ini->put(section, "balance_ankle_pitch_gain", BALANCE_ANKLE_PITCH_GAIN);
    ini->put(section, "balance_hip_roll_gain", BALANCE_HIP_ROLL_GAIN);
    ini->put(section, "balance_ankle_roll_gain", BALANCE_ANKLE_ROLL_GAIN);

    ini->put(section, "p_gain", P_GAIN);
    ini->put(section, "i_gain", I_GAIN);
    ini->put(section, "d_gain", D_GAIN);
}


void Walking::update_param_time() {
    m_PeriodTime = PERIOD_TIME;
    m_DSP_Ratio = DSP_RATIO;
    m_SSP_Ratio = 1 - DSP_RATIO;

    m_X_Swap_PeriodTime = m_PeriodTime / 2;
    m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Y_Swap_PeriodTime = m_PeriodTime;
    m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Z_Swap_PeriodTime = m_PeriodTime / 2;
    m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
    m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

    m_SSP_Time = m_PeriodTime * m_SSP_Ratio;
    m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;

    m_Phase_Time1 = (m_SSP_Time_End_L + m_SSP_Time_Start_L) / 2;
    m_Phase_Time2 = (m_SSP_Time_Start_R + m_SSP_Time_End_L) / 2;
    m_Phase_Time3 = (m_SSP_Time_End_R + m_SSP_Time_Start_R) / 2;

    m_Pelvis_Offset = PELVIS_OFFSET * MX28::RATIO_ANGLE2VALUE;
    m_Pelvis_Swing = m_Pelvis_Offset * 0.35;
    m_Arm_Swing_Gain = ARM_SWING_GAIN;
}


void Walking::update_param_move() {
    // Forward/Back
    m_X_Move_Amplitude = X_MOVE_AMPLITUDE;
    m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO;

    // Right/Left
    m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
    if (m_Y_Move_Amplitude > 0)
        m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;
    else
        m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
    m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;

    m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
    m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
    m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;
    m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;

    // Direction
    if (A_MOVE_AIM_ON == false) {
        m_A_Move_Amplitude = radians(A_MOVE_AMPLITUDE) / 2;
        if (m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
    } else {
        m_A_Move_Amplitude = -radians(A_MOVE_AMPLITUDE) / 2;
        if (m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
    }
}


void Walking::update_param_balance() {
    m_X_Offset = X_OFFSET;
    m_Y_Offset = Y_OFFSET;
    m_Z_Offset = Z_OFFSET;
    m_R_Offset = radians(R_OFFSET);
    m_P_Offset = radians(P_OFFSET);
    m_A_Offset = radians(A_OFFSET);
    m_Hip_Pitch_Offset = HIP_PITCH_OFFSET * MX28::RATIO_ANGLE2VALUE;
}


void Walking::Initialize() {
    X_MOVE_AMPLITUDE = 0;
    Y_MOVE_AMPLITUDE = 0;
    A_MOVE_AMPLITUDE = 0;

    m_Body_Swing_Y = 0;
    m_Body_Swing_Z = 0;

    m_X_Swap_Phase_Shift = pi;
    m_X_Swap_Amplitude_Shift = 0;
    m_X_Move_Phase_Shift = half_pi;
    m_X_Move_Amplitude_Shift = 0;
    m_Y_Swap_Phase_Shift = 0;
    m_Y_Swap_Amplitude_Shift = 0;
    m_Y_Move_Phase_Shift = half_pi;
    m_Z_Swap_Phase_Shift = half_pi * 3;
    m_Z_Move_Phase_Shift = half_pi;
    m_A_Move_Phase_Shift = half_pi;

    m_Ctrl_Running = false;
    m_Real_Running = false;
    m_Time = 0;
    update_param_time();
    update_param_move();

    Process();
}


void Walking::Start() {
    m_Ctrl_Running = true;
    m_Real_Running = true;
}


void Walking::Stop() {
    m_Ctrl_Running = false;
}


bool Walking::IsRunning() {
    return m_Real_Running;
}


void Walking::Process() {
    float x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
    float x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
    float x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
    float pelvis_offset_r, pelvis_offset_l;
    float angle[14], ep[12];
    float offset;
    float TIME_UNIT = MotionModule::TIME_UNIT;
    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    int dir[14] = {-1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1};
    float initAngle[14] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -48.345, 41.313};

    int outValue[14];

    // Update walk parameters
    if (m_Time == 0) {
        update_param_time();
        m_Phase = PHASE0;
        if (!m_Ctrl_Running) {
            if (m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0) {
                m_Real_Running = false;
            } else {
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    } else if (m_Time >= (m_Phase_Time1 - TIME_UNIT / 2) && m_Time < (m_Phase_Time1 + TIME_UNIT / 2)) {
        update_param_move();
        m_Phase = PHASE1;
    } else if (m_Time >= (m_Phase_Time2 - TIME_UNIT / 2) && m_Time < (m_Phase_Time2 + TIME_UNIT / 2)) {
        update_param_time();
        m_Time = m_Phase_Time2;
        m_Phase = PHASE2;
        if (!m_Ctrl_Running) {
            if (m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0) {
                m_Real_Running = false;
            } else {
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    } else if (m_Time >= (m_Phase_Time3 - TIME_UNIT / 2) && m_Time < (m_Phase_Time3 + TIME_UNIT / 2)) {
        update_param_move();
        m_Phase = PHASE3;
    }
    update_param_balance();

    // Compute endpoints
    x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
    y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
    z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
    a_swap = 0;
    b_swap = 0;
    c_swap = 0;

    if (m_Time <= m_SSP_Time_Start_L) {
        x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude,
                        m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude,
                        m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude,
                        m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude,
                        -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude,
                        -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude,
                        -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
        m_left_start = true;
    } else if (m_Time <= m_SSP_Time_End_L) {
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude,
                        m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude,
                        m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_Time, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude,
                        m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude,
                        -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude,
                        -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude,
                        -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime,
                               m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_L,
                               m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime,
                               m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_L,
                               -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
        m_left_end = true;
    } else if (m_Time <= m_SSP_Time_Start_R) {
        x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude,
                        m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude,
                        m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude,
                        m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude,
                        -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude,
                        -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude,
                        -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
        m_right_start = true;
    } else if (m_Time <= m_SSP_Time_End_R) {
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_Time, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime,
                               m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_R,
                               m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime,
                               m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_R,
                               -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
        m_right_end = true;
    } else {
        x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime,
                        m_X_Move_Phase_Shift + two_pi / m_X_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime,
                        m_Y_Move_Phase_Shift + two_pi / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime,
                        m_Z_Move_Phase_Shift + two_pi / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude,
                        m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime,
                        m_A_Move_Phase_Shift + two_pi / m_A_Move_PeriodTime * m_SSP_Time_Start_R + pi,
                        -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }

    a_move_l = 0;
    b_move_l = 0;
    a_move_r = 0;
    b_move_r = 0;

    ep[0] = x_swap + x_move_r + m_X_Offset;
    ep[1] = y_swap + y_move_r - m_Y_Offset / 2;
    ep[2] = z_swap + z_move_r + m_Z_Offset;
    ep[3] = a_swap + a_move_r - m_R_Offset / 2;
    ep[4] = b_swap + b_move_r + m_P_Offset;
    ep[5] = c_swap + c_move_r - m_A_Offset / 2;
    ep[6] = x_swap + x_move_l + m_X_Offset;
    ep[7] = y_swap + y_move_l + m_Y_Offset / 2;
    ep[8] = z_swap + z_move_l + m_Z_Offset;
    ep[9] = a_swap + a_move_l + m_R_Offset / 2;
    ep[10] = b_swap + b_move_l + m_P_Offset;
    ep[11] = c_swap + c_move_l + m_A_Offset / 2;

    if(m_left_end && m_left_start) {
        if(X_MOVE_AMPLITUDE != 0 || Y_MOVE_AMPLITUDE != 0) {
            m_odo_x += m_left_odo_x;
            m_odo_y += m_left_odo_y;
        }
        m_odo_theta += m_left_odo_theta;
        m_left_end = false;
        m_left_start = false;
        m_right_end = false;
    }
    else if(m_right_end && m_right_start) {
        if(X_MOVE_AMPLITUDE != 0 || Y_MOVE_AMPLITUDE != 0) {
            m_odo_x += m_right_odo_x;
            m_odo_y += m_right_odo_y;
        }
        m_odo_theta += m_right_odo_theta;
        m_right_end = false;
        m_right_start = false;
        m_left_end = false;
    } else {
        if(X_MOVE_AMPLITUDE > 0) {
            m_left_odo_x = -m_odo_x_factor * ep[6];
        }
        else {
            m_left_odo_x = m_odo_x_factor * ep[6];
        }
        m_left_odo_y = -m_odo_y_factor * ep[7];
        m_left_odo_theta = -m_odo_a_factor * ep[11];

        if(X_MOVE_AMPLITUDE > 0) {
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
    if (m_Time <= m_SSP_Time_End_L) {
        m_Body_Swing_Y = -ep[7];
        m_Body_Swing_Z = ep[8];
    } else {
        m_Body_Swing_Y = -ep[1];
        m_Body_Swing_Z = ep[2];
    }
    m_Body_Swing_Z -= Kinematics::LEG_LENGTH;

    // Compute arm swing
    if (m_X_Move_Amplitude == 0) {
        angle[12] = 0; // Right
        angle[13] = 0; // Left
    } else {
        angle[12] = wsin(m_Time, m_PeriodTime, pi * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
        angle[13] = wsin(m_Time, m_PeriodTime, pi * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    }

    if (m_Real_Running) {
        m_Time += TIME_UNIT;
        if (m_Time >= m_PeriodTime)
            m_Time = 0;
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
        offset = (float) dir[i] * angle[i] * MX28::RATIO_ANGLE2VALUE;
        if (i == 1) // R_HIP_ROLL
            offset += (float) dir[i] * (pelvis_offset_r /** cos(ep[5])*/ - HIP_PITCH_OFFSET * sin(ep[5]) * MX28::RATIO_ANGLE2VALUE);
        else if (i == 7) // L_HIP_ROLL
            offset += (float) dir[i] * (pelvis_offset_l /** cos(ep[11])*/ - HIP_PITCH_OFFSET * sin(ep[11]) * MX28::RATIO_ANGLE2VALUE);
        else if (i == 2) // R_HIP_PITCH
            offset += (float) dir[i] * (/*-pelvis_offset_r * sin(ep[5])*/ - HIP_PITCH_OFFSET * cos(ep[5]) ) * MX28::RATIO_ANGLE2VALUE;
        else if (i == 8) // L_HIP_PITCH
            offset += (float) dir[i] * (/*-pelvis_offset_l * sin(ep[11])*/ - HIP_PITCH_OFFSET * cos(ep[11])) * MX28::RATIO_ANGLE2VALUE;

        outValue[i] = MX28::Angle2Value(initAngle[i]) + (int) offset;
    }

//    // Compute motor value
//    for (int i = 0; i < 14; i++) {
//        offset = (float) dir[i] * angle[i] * MX28::RATIO_ANGLE2VALUE;
//        if (i == 1) // R_HIP_ROLL
//            offset += (float) dir[i] * pelvis_offset_r;
//        else if (i == 7) // L_HIP_ROLL
//            offset += (float) dir[i] * pelvis_offset_l;
//        else if (i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
//            offset -= (float) dir[i] * HIP_PITCH_OFFSET * MX28::RATIO_ANGLE2VALUE;
//
//        outValue[i] = MX28::Angle2Value(initAngle[i]) + (int) offset;
//    }

    // adjust balance offset
    if (BALANCE_ENABLE) {
        float rlGyroErr = MotionStatus::RL_GYRO;
        float fbGyroErr = MotionStatus::FB_GYRO;
        outValue[1] += (int) (dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN * 4); // R_HIP_ROLL
        outValue[7] += (int) (dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN * 4); // L_HIP_ROLL

        outValue[3] -= (int) (dir[3] * fbGyroErr * BALANCE_KNEE_GAIN * 4); // R_KNEE
        outValue[9] -= (int) (dir[9] * fbGyroErr * BALANCE_KNEE_GAIN * 4); // L_KNEE

        outValue[4] -= (int) (dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN * 4); // R_ANKLE_PITCH
        outValue[10] -= (int) (dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN * 4); // L_ANKLE_PITCH

        outValue[5] -= (int) (dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN * 4); // R_ANKLE_ROLL
        outValue[11] -= (int) (dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN * 4); // L_ANKLE_ROLL
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
    m_Joint.SetAngle(JointData::ID_HEAD_PAN, A_MOVE_AMPLITUDE);

    for (int id = JointData::ID_R_HIP_YAW; id <= JointData::ID_L_ANKLE_ROLL; id++) {
        m_Joint.SetPGain(id, P_GAIN);
        m_Joint.SetIGain(id, I_GAIN);
        m_Joint.SetDGain(id, D_GAIN);
    }
}

Pose2D Walking::GetOdoOffset() {
    Pose2D offset(m_odo_x, m_odo_y, m_odo_theta);
    m_odo_x = 0;
    m_odo_y = 0;
    m_odo_theta = 0;
    return offset;
}

float Walking::GetXMoveAmplitude() const {
    return X_MOVE_AMPLITUDE;
}

float Walking::GetYMoveAmplitude() const {
    return Y_MOVE_AMPLITUDE;
}

float Walking::GetAMoveAmplitude() const {
    return A_MOVE_AMPLITUDE;
}

bool Walking::IsAimMode() const {
    return A_MOVE_AIM_ON;
}

void Walking::SetMoveAmplitude(float x, float y, float a, bool aim_mode) {
    X_MOVE_AMPLITUDE = x;
    Y_MOVE_AMPLITUDE = y;
    A_MOVE_AMPLITUDE = a;
    A_MOVE_AIM_ON = aim_mode;
    GO_TO_MODE_ON = false;
}

int Walking::GetCurrentPhase() { return m_Phase; }

float Walking::GetBodySwingY() { return m_Body_Swing_Y; }

float Walking::GetBodySwingZ() { return m_Body_Swing_Z; }

Pose2D Walking::GetOdo() {
    return m_odometry_collector.GetPose();
}

void Walking::ResetOdo(Pose2D pose) {
    m_odometry_collector.Reset();
}

void Walking::SetOdo(Pose2D pose) {
    m_odometry_collector.SetPose(pose);
}
