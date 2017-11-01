/*
 *   MotionStatus.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include "joint_data_t.h"


namespace drwn {
    enum {
        BACKWARD = -1,
        STANDUP = 0,
        FORWARD = 1
    };

    class motion_status_t {
    private:

    public:
        static const int FALLEN_F_LIMIT = 390;
        static const int FALLEN_B_LIMIT = 580;
        static const int FALLEN_MAX_COUNT = 30;

        static joint_data_t m_current_joints;
        static int FB_GYRO;
        static int RL_GYRO;
        static int FB_ACCEL;
        static int RL_ACCEL;

        static int BUTTON;
        static int FALLEN;
    };
}

