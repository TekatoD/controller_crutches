/*
 *   Head.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <log/trivial_logger_t.h>
#include "hw/MX28_t.h"
#include "motion/kinematics_t.h"
#include "motion/motion_status_t.h"
#include "motion/modules/head_t.h"

using namespace drwn;

/*initialization of the head*/


head_t::head_t() {
    m_Pan_p_gain = 0.1;
    m_Pan_d_gain = 0.22;

    m_Tilt_p_gain = 0.1;
    m_Tilt_d_gain = 0.22;

    m_left_limit = 70;
    m_right_limit = -70;
    m_top_limit = kinematics_t::EYE_TILT_OFFSET_ANGLE;
    m_bottom_limit = kinematics_t::EYE_TILT_OFFSET_ANGLE - 65;

    m_Pan_Home = 0.0;
    m_Tilt_Home = kinematics_t::EYE_TILT_OFFSET_ANGLE - 30.0;

    joint.set_enable_head_only(true);
}


head_t::~head_t() {
}


void head_t::check_limit() {
    if (m_pan_angle > m_left_limit)
        m_pan_angle = m_left_limit;
    else if (m_pan_angle < m_right_limit)
        m_pan_angle = m_right_limit;

    if (m_tilt_angle > m_top_limit)
        m_tilt_angle = m_top_limit;
    else if (m_tilt_angle < m_bottom_limit)
        m_tilt_angle = m_bottom_limit;
}


void head_t::initialize() {
    m_pan_angle = motion_status_t::current_joints.get_angle(joint_data_t::ID_HEAD_PAN);
    m_tilt_angle = -motion_status_t::current_joints.get_angle(joint_data_t::ID_HEAD_TILT);
    check_limit();

    init_tracking();
    move_to_home();
}


float head_t::get_left_limit() const {
    return m_left_limit;
}

void head_t::set_left_limit(float left_limit) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: left_limit = " << left_limit;
    }
    m_left_limit = left_limit;
}

float head_t::get_right_limit() const {
    return m_right_limit;
}

void head_t::set_right_limit(float right_limit) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: right_limit = " << right_limit;
    }
    m_right_limit = right_limit;
}

float head_t::get_top_limit() const {
    return m_top_limit;
}

void head_t::set_top_limit(float top_limit) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: top_limit = " << top_limit;
    }
    head_t::m_top_limit = top_limit;
}

float head_t::get_bottom_limit() const {
    return m_bottom_limit;
}

void head_t::set_bottom_limit(float bottom_limit) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: bottom_limit = " << bottom_limit;
    }
    m_bottom_limit = bottom_limit;
}

float head_t::get_pan_home() const {
    return m_Pan_Home;
}

void head_t::set_pan_home(float pan_home) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: pan_home = " << pan_home;
    }
    m_Pan_Home = pan_home;
}

float head_t::get_tilt_home() const {
    return m_Tilt_Home;
}

void head_t::set_tilt_home(float tilt_home) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: tilt_home = " << tilt_home;
    }
    m_Tilt_Home = tilt_home;
}

float head_t::get_pan_p_gain() const {
    return m_Pan_p_gain;
}

void head_t::set_pan_p_gain(float pan_p_gain) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: pan_p_gain = " << pan_p_gain;
    }
    m_Pan_p_gain = pan_p_gain;
}

float head_t::get_pan_d_gain() const {
    return m_Pan_d_gain;
}

void head_t::set_pan_d_gain(float pan_d_gain) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: pan_d_gain = " << pan_d_gain;
    }
    m_Pan_d_gain = pan_d_gain;
}

float head_t::get_tilt_p_gain() const {
    return m_Tilt_p_gain;
}

void head_t::set_tilt_p_gain(float tilt_p_gain) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: tilt_p_gain = " << tilt_p_gain;
    }
    m_Tilt_p_gain = tilt_p_gain;
}

float head_t::get_tilt_d_gain() const {
    return m_Tilt_d_gain;
}

void head_t::set_tilt_d_gain(float tilt_d_gain) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: tilt_d_gain = " << tilt_d_gain;
    }
    m_Tilt_d_gain = tilt_d_gain;
}


void head_t::move_to_home() {
    move_by_angle(m_Pan_Home, m_Tilt_Home);
}


void head_t::move_by_angle(float pan, float tilt) {
    m_pan_angle = pan;
    m_tilt_angle = tilt;

    check_limit();
}


void head_t::move_by_angle_offset(float pan, float tilt) {
    move_by_angle(m_pan_angle + pan, m_tilt_angle + tilt);
}


void head_t::init_tracking() {
    m_Pan_err = 0;
    m_Pan_err_diff = 0;
    m_Tilt_err = 0;
    m_Tilt_err_diff = 0;
}


void head_t::move_tracking(point2d_t err) {
    m_Pan_err_diff = err.X - m_Pan_err;
    m_Pan_err = err.X;

    m_Tilt_err_diff = err.Y - m_Tilt_err;
    m_Tilt_err = err.Y;

    move_tracking();
}


void head_t::move_tracking() {
    float pOffset, dOffset;

    pOffset = m_Pan_err * m_Pan_p_gain;
    pOffset *= pOffset;
    if (m_Pan_err < 0)
        pOffset = -pOffset;
    dOffset = m_Pan_err_diff * m_Pan_d_gain;
    dOffset *= dOffset;
    if (m_Pan_err_diff < 0)
        dOffset = -dOffset;
    m_pan_angle += (pOffset + dOffset);

    pOffset = m_Tilt_err * m_Tilt_p_gain;
    pOffset *= pOffset;
    if (m_Tilt_err < 0)
        pOffset = -pOffset;
    dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
    dOffset *= dOffset;
    if (m_Tilt_err_diff < 0)
        dOffset = -dOffset;
    m_tilt_angle += (pOffset + dOffset);

    check_limit();
}


void head_t::process() {
    if (joint.get_enable(joint_data_t::ID_HEAD_PAN) == true)
        joint.set_angle(joint_data_t::ID_HEAD_PAN, m_pan_angle);

    if (joint.get_enable(joint_data_t::ID_HEAD_TILT) == true)
        joint.set_angle(joint_data_t::ID_HEAD_TILT, m_tilt_angle);
}

bool head_t::is_debug_enabled() const {
    return m_debug;
}

void head_t::enable_debug(bool debug) {
    m_debug = debug;
}
