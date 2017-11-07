/*
 *   JointData.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "hw/MX28_t.h"
#include "motion/joint_data_t.h"
#include "motion/motion_manager_t.h"

using namespace drwn;


joint_data_t::joint_data_t() {
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        m_enable[i] = true;
        m_value[i] = MX28_t::CENTER_VALUE;
        m_angle[i] = 0.0;
        m_p_gain[i] = P_GAIN_DEFAULT;
        m_i_gain[i] = I_GAIN_DEFAULT;
        m_d_gain[i] = D_GAIN_DEFAULT;
    }
}


joint_data_t::~joint_data_t() {
}


void joint_data_t::set_enable(int id, bool enable) {
    m_enable[id] = enable;
}


void joint_data_t::set_enable(int id, bool enable, bool exclusive) {
#ifndef WEBOTS // Because MotionManager is not included in the lite version of the Framework used in the simulation
    if (enable && exclusive) motion_manager_t::get_instance()->set_joint_disable(id);
#endif
    m_enable[id] = enable;
}


void joint_data_t::set_enable_head_only(bool enable) {
    set_enable_head_only(enable, false);
}


void joint_data_t::set_enable_head_only(bool enable, bool exclusive) {
    set_enable(ID_HEAD_PAN, enable, exclusive);
    set_enable(ID_HEAD_TILT, enable, exclusive);
}


void joint_data_t::set_enable_right_arm_only(bool enable) {
    set_enable_right_arm_only(enable, false);
}


void joint_data_t::set_enable_right_arm_only(bool enable, bool exclusive) {
    set_enable(ID_R_SHOULDER_PITCH, enable, exclusive);
    set_enable(ID_R_SHOULDER_ROLL, enable, exclusive);
    set_enable(ID_R_ELBOW, enable, exclusive);
}


void joint_data_t::set_enable_left_arm_only(bool enable) {
    set_enable_left_arm_only(enable, false);
}


void joint_data_t::set_enable_left_arm_only(bool enable, bool exclusive) {
    set_enable(ID_L_SHOULDER_PITCH, enable, exclusive);
    set_enable(ID_L_SHOULDER_ROLL, enable, exclusive);
    set_enable(ID_L_ELBOW, enable, exclusive);
}


void joint_data_t::set_enable_right_leg_only(bool enable) {
    set_enable_right_leg_only(enable, false);
}


void joint_data_t::set_enable_right_leg_only(bool enable, bool exclusive) {
    set_enable(ID_R_HIP_YAW, enable, exclusive);
    set_enable(ID_R_HIP_ROLL, enable, exclusive);
    set_enable(ID_R_HIP_PITCH, enable, exclusive);
    set_enable(ID_R_KNEE, enable, exclusive);
    set_enable(ID_R_ANKLE_PITCH, enable, exclusive);
    set_enable(ID_R_ANKLE_ROLL, enable, exclusive);
}


void joint_data_t::set_enable_left_leg_only(bool enable) {
    set_enable_left_leg_only(enable, false);
}


void joint_data_t::set_enable_left_leg_only(bool enable, bool exclusive) {
    set_enable(ID_L_HIP_YAW, enable, exclusive);
    set_enable(ID_L_HIP_ROLL, enable, exclusive);
    set_enable(ID_L_HIP_PITCH, enable, exclusive);
    set_enable(ID_L_KNEE, enable, exclusive);
    set_enable(ID_L_ANKLE_PITCH, enable, exclusive);
    set_enable(ID_L_ANKLE_ROLL, enable, exclusive);
}


void joint_data_t::set_enable_upper_body_without_head(bool enable) {
    set_enable_upper_body_without_head(enable, false);
}


void joint_data_t::set_enable_upper_body_without_head(bool enable, bool exclusive) {
    set_enable_right_arm_only(enable, exclusive);
    set_enable_left_arm_only(enable, exclusive);
}


void joint_data_t::set_enable_lower_body(bool enable) {
    set_enable_lower_body(enable, false);
}


void joint_data_t::set_enable_lower_body(bool enable, bool exclusive) {
    set_enable_right_leg_only(enable, exclusive);
    set_enable_left_leg_only(enable, exclusive);
}


void joint_data_t::set_enable_body_without_head(bool enable) {
    set_enable_body_without_head(enable, false);
}


void joint_data_t::set_enable_body_without_head(bool enable, bool exclusive) {
    set_enable_right_arm_only(enable, exclusive);
    set_enable_left_arm_only(enable, exclusive);
    set_enable_right_leg_only(enable, exclusive);
    set_enable_left_leg_only(enable, exclusive);
}


void joint_data_t::set_enable_body(bool enable) {
    set_enable_body(enable, false);
}


void joint_data_t::set_enable_body(bool enable, bool exclusive) {
    for (int id = 1; id < NUMBER_OF_JOINTS; id++)
        set_enable(id, enable, exclusive);
}


bool joint_data_t::get_enable(int id) {
    return m_enable[id];
}


void joint_data_t::set_value(int id, int value) {
    if (value < MX28_t::MIN_VALUE)
        value = MX28_t::MIN_VALUE;
    else if (value >= MX28_t::MAX_VALUE)
        value = MX28_t::MAX_VALUE;

    m_value[id] = value;
    m_angle[id] = MX28_t::value_2_angle(value);
}


int joint_data_t::get_value(int id) {
    return m_value[id];
}


void joint_data_t::set_angle(int id, float angle) {
    if (angle < MX28_t::MIN_DEGREES)
        angle = MX28_t::MIN_DEGREES;
    else if (angle > MX28_t::MAX_DEGREES)
        angle = MX28_t::MAX_DEGREES;

    m_angle[id] = angle;
    m_value[id] = MX28_t::angle_2_value(angle);
}


float joint_data_t::set_angle(int id) {
    return m_angle[id];
}


void joint_data_t::set_radian(int id, float radian) {
    set_angle(id, radian * (180.0 / 3.141592));
}


float joint_data_t::get_radian(int id) {
    return set_angle(id) * (180.0 / 3.141592);
}

