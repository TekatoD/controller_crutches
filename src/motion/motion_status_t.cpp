/*
 *   MotionStatus.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "motion/motion_status_t.h"

using namespace drwn;

joint_data_t motion_status_t::m_current_joints;
int motion_status_t::FB_GYRO(0);
int motion_status_t::RL_GYRO(0);
int motion_status_t::FB_ACCEL(0);
int motion_status_t::RL_ACCEL(0);

int motion_status_t::BUTTON(0);
int motion_status_t::FALLEN(0);
