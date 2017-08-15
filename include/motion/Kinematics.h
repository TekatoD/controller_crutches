/*
 *   Kinematics.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <cstring>
#include "math/Eigen.h"
#include "math/Matrix.h"
#include "JointData.h"

namespace Robot {
    class Kinematics {
    private:
        Kinematics();

    public:
        static bool ComputeLegInverseKinematics(float* out, float x, float y, float z, float a, float b, float c);

        static void ComputeLegForwardKinematics(Matrix4x4f& out, float pelvis, float tight_roll, float tight_pitch,
                                                float knee_pitch, float ankle_pitch, float ankle_roll);

        static void ComputeHeadForwardKinematics(Matrix4x4f& out, float pan, float tilt);

    public:
        static constexpr float CAMERA_DISTANCE = 33.2f; //mm
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

        ~Kinematics();

    };
}

#endif
