/*
 *   Head.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <log/Logger.h>
#include "hw/MX28.h"
#include "motion/Kinematics.h"
#include "motion/MotionStatus.h"
#include "motion/modules/Head.h"

using namespace Robot;

/*initialization of the head*/


Head::Head() {
    m_Pan_p_gain = 0.1;
    m_Pan_d_gain = 0.22;

    m_Tilt_p_gain = 0.1;
    m_Tilt_d_gain = 0.22;

    m_LeftLimit = 70;
    m_RightLimit = -70;
    m_TopLimit = Kinematics::EYE_TILT_OFFSET_ANGLE;
    m_BottomLimit = Kinematics::EYE_TILT_OFFSET_ANGLE - 65;

    m_Pan_Home = 0.0;
    m_Tilt_Home = Kinematics::EYE_TILT_OFFSET_ANGLE - 30.0;

    m_Joint.SetEnableHeadOnly(true);
}


Head::~Head() {
}


void Head::CheckLimit() {
    if (m_PanAngle > m_LeftLimit)
        m_PanAngle = m_LeftLimit;
    else if (m_PanAngle < m_RightLimit)
        m_PanAngle = m_RightLimit;

    if (m_TiltAngle > m_TopLimit)
        m_TiltAngle = m_TopLimit;
    else if (m_TiltAngle < m_BottomLimit)
        m_TiltAngle = m_BottomLimit;
}


void Head::Initialize() {
    m_PanAngle = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
    m_TiltAngle = -MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
    CheckLimit();

    InitTracking();
    MoveToHome();
}


float Head::GetLeftLimit() const {
    return m_LeftLimit;
}

void Head::SetLeftLimit(float left_limit) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: left_limit = " << left_limit;
    }
    m_LeftLimit = left_limit;
}

float Head::GetRightLimit() const {
    return m_RightLimit;
}

void Head::SetRightLimit(float right_limit) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: right_limit = " << right_limit;
    }
    m_RightLimit = right_limit;
}

float Head::GetTopLimit() const {
    return m_TopLimit;
}

void Head::SetTopLimit(float top_limit) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: top_limit = " << top_limit;
    }
    Head::m_TopLimit = top_limit;
}

float Head::GetBottomLimit() const {
    return m_BottomLimit;
}

void Head::SetBottomLimit(float bottom_limit) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: bottom_limit = " << bottom_limit;
    }
    m_BottomLimit = bottom_limit;
}

float Head::GetPanHome() const {
    return m_Pan_Home;
}

void Head::SetPanHome(float pan_home) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: pan_home = " << pan_home;
    }
    m_Pan_Home = pan_home;
}

float Head::GetTiltHome() const {
    return m_Tilt_Home;
}

void Head::SetTiltHome(float tilt_home) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: tilt_home = " << tilt_home;
    }
    m_Tilt_Home = tilt_home;
}

float Head::GetPanPGain() const {
    return m_Pan_p_gain;
}

void Head::SetPanPGain(float pan_p_gain) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: pan_p_gain = " << pan_p_gain;
    }
    m_Pan_p_gain = pan_p_gain;
}

float Head::GetPanDGain() const {
    return m_Pan_d_gain;
}

void Head::SetPanDGain(float pan_d_gain) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: pan_d_gain = " << pan_d_gain;
    }
    m_Pan_d_gain = pan_d_gain;
}

float Head::GetTiltPGain() const {
    return m_Tilt_p_gain;
}

void Head::SetTiltPGain(float tilt_p_gain) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: tilt_p_gain = " << tilt_p_gain;
    }
    m_Tilt_p_gain = tilt_p_gain;
}

float Head::GetTiltDGain() const {
    return m_Tilt_d_gain;
}

void Head::SetTiltDGain(float tilt_d_gain) {
    if(m_debug) {
        LOG_DEBUG << "HEAD: tilt_d_gain = " << tilt_d_gain;
    }
    m_Tilt_d_gain = tilt_d_gain;
}


void Head::MoveToHome() {
    MoveByAngle(m_Pan_Home, m_Tilt_Home);
}


void Head::MoveByAngle(float pan, float tilt) {
    m_PanAngle = pan;
    m_TiltAngle = tilt;

    CheckLimit();
}


void Head::MoveByAngleOffset(float pan, float tilt) {
    MoveByAngle(m_PanAngle + pan, m_TiltAngle + tilt);
}


void Head::InitTracking() {
    m_Pan_err = 0;
    m_Pan_err_diff = 0;
    m_Tilt_err = 0;
    m_Tilt_err_diff = 0;
}


void Head::MoveTracking(Point2D err) {
    m_Pan_err_diff = err.X - m_Pan_err;
    m_Pan_err = err.X;

    m_Tilt_err_diff = err.Y - m_Tilt_err;
    m_Tilt_err = err.Y;

    MoveTracking();
}


void Head::MoveTracking() {
    float pOffset, dOffset;

    pOffset = m_Pan_err * m_Pan_p_gain;
    pOffset *= pOffset;
    if (m_Pan_err < 0)
        pOffset = -pOffset;
    dOffset = m_Pan_err_diff * m_Pan_d_gain;
    dOffset *= dOffset;
    if (m_Pan_err_diff < 0)
        dOffset = -dOffset;
    m_PanAngle += (pOffset + dOffset);

    pOffset = m_Tilt_err * m_Tilt_p_gain;
    pOffset *= pOffset;
    if (m_Tilt_err < 0)
        pOffset = -pOffset;
    dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
    dOffset *= dOffset;
    if (m_Tilt_err_diff < 0)
        dOffset = -dOffset;
    m_TiltAngle += (pOffset + dOffset);

    CheckLimit();
}


void Head::Process() {
    if (m_Joint.GetEnable(JointData::ID_HEAD_PAN) == true)
        m_Joint.SetAngle(JointData::ID_HEAD_PAN, m_PanAngle);

    if (m_Joint.GetEnable(JointData::ID_HEAD_TILT) == true)
        m_Joint.SetAngle(JointData::ID_HEAD_TILT, m_TiltAngle);
}

bool Head::IsDebugEnabled() const {
    return m_debug;
}

void Head::EnableDebug(bool debug) {
    m_debug = debug;
}
