/*
 *   MotionStatus.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include "joint_data_t.h"


namespace drwn {
    enum class fall_type_t {
        BACKWARD = -1,
        STANDUP = 0,
        FORWARD = 1
    };

    struct motion_status_t {
        static const int FALLEN_F_LIMIT = 390;
        static const int FALLEN_B_LIMIT = 580;
        static const int FALLEN_MAX_COUNT = 30;

        static joint_data_t current_joints;
        static int x_gyro;
        static int y_gyro;
        static int x_accel;
        static int y_accel;

        static int button;
        static fall_type_t fall_type;
    };
}

