/*
 *   Head.cpp
 *
 *   Author: ROBOTIS
 *
 */

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
