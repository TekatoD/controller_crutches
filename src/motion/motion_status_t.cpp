/*
 *   MotionStatus.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "motion/motion_status_t.h"

using namespace drwn;

joint_data_t motion_status_t::current_joints;
int motion_status_t::y_gyro(0);
int motion_status_t::x_gyro(0);
int motion_status_t::y_accel(0);
int motion_status_t::x_accel(0);

int motion_status_t::button(0);
fall_type_t motion_status_t::fall_type(fall_type_t::STANDUP);
