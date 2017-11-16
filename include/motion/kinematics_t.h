/*
 *   Kinematics.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include <cstring>
#include "math/eigen.h"
#include "math/matrix_t.h"
#include "joint_data_t.h"

namespace drwn {
    class kinematics_t {
    public:
        static bool compute_leg_inverse_kinematics(float* out, float x, float y, float z, float a, float b, float c);

        static void compute_leg_forward_kinematics(matrix4x4f_t& out, float pelvis, float tight_roll,
                                                   float tight_pitch,
                                                   float knee_pitch, float ankle_pitch, float ankle_roll);

        static void compute_head_forward_kinematics(matrix4x4f_t& out, float pan, float tilt);

    public:
        static constexpr float CAMERA_OFFSET_X = 33.2f; //mm
        static constexpr float CAMERA_OFFSET_Z = 34.4f; //mm
        static constexpr float EYE_TILT_OFFSET_ANGLE = 40.0f; //degree
        static constexpr float LEG_SIDE_OFFSET = 37.0f; //mm
        static constexpr float THIGH_LENGTH = 93.0f; //mm
        static constexpr float CALF_LENGTH = 93.0f; //mm
        static constexpr float ANKLE_LENGTH = 33.5f; //mm
        static constexpr float LEG_LENGTH = 219.5f; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

        static constexpr float CW_LIMIT_R_SHOULDER_ROLL = -75.0f; // degree
        static constexpr float CCW_LIMIT_R_SHOULDER_ROLL = 135.0f; // degree
        static constexpr float CW_LIMIT_L_SHOULDER_ROLL = -135.0f; // degree
        static constexpr float CCW_LIMIT_L_SHOULDER_ROLL = 75.0f; // degree
        static constexpr float CW_LIMIT_R_ELBOW = -95.0f; // degree
        static constexpr float CCW_LIMIT_R_ELBOW = 70.0f; // degree
        static constexpr float CW_LIMIT_L_ELBOW = -70.0f; // degree
        static constexpr float CCW_LIMIT_L_ELBOW = 95.0f; // degree
        static constexpr float CW_LIMIT_R_HIP_YAW = -123.0f; // degree
        static constexpr float CCW_LIMIT_R_HIP_YAW = 53.0f; // degree
        static constexpr float CW_LIMIT_L_HIP_YAW = -53.0f; // degree
        static constexpr float CCW_LIMIT_L_HIP_YAW = 123.0f; // degree
        static constexpr float CW_LIMIT_R_HIP_ROLL = -45.0f; // degree
        static constexpr float CCW_LIMIT_R_HIP_ROLL = 59.0f; // degree
        static constexpr float CW_LIMIT_L_HIP_ROLL = -59.0f; // degree
        static constexpr float CCW_LIMIT_L_HIP_ROLL = 45.0f; // degree
        static constexpr float CW_LIMIT_R_HIP_PITCH = -100.0f; // degree
        static constexpr float CCW_LIMIT_R_HIP_PITCH = 29.0f; // degree
        static constexpr float CW_LIMIT_L_HIP_PITCH = -29.0f; // degree
        static constexpr float CCW_LIMIT_L_HIP_PITCH = 100.0f; // degree
        static constexpr float CW_LIMIT_R_KNEE = -6.0f; // degree
        static constexpr float CCW_LIMIT_R_KNEE = 130.0f; // degree
        static constexpr float CW_LIMIT_L_KNEE = -130.0f; // degree
        static constexpr float CCW_LIMIT_L_KNEE = 6.0f; // degree
        static constexpr float CW_LIMIT_R_ANKLE_PITCH = -72.0f; // degree
        static constexpr float CCW_LIMIT_R_ANKLE_PITCH = 80.0f; // degree
        static constexpr float CW_LIMIT_L_ANKLE_PITCH = -80.0f; // degree
        static constexpr float CCW_LIMIT_L_ANKLE_PITCH = 72.0f; // degree
        static constexpr float CW_LIMIT_R_ANKLE_ROLL = -44.0f; // degree
        static constexpr float CCW_LIMIT_R_ANKLE_ROLL = 63.0f; // degree
        static constexpr float CW_LIMIT_L_ANKLE_ROLL = -63.0f; // degree
        static constexpr float CCW_LIMIT_L_ANKLE_ROLL = 44.0f; // degree
        static constexpr float CW_LIMIT_HEAD_PAN = -90.0f; // degree
        static constexpr float CCW_LIMIT_HEAD_PAN = 90.0f; // degree
        static constexpr float CW_LIMIT_HEAD_TILT = -25.0f; // degree
        static constexpr float CCW_LIMIT_HEAD_TILT = 55.0f; // degree

    };
}
