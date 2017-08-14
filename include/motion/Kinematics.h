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

        static Matrix4x4f ComputeLegForwardKinematics(float pelvis, float tight_roll, float tight_pitch,
                                                      float knee_pitch, float ankle_pitch, float ankle_roll);

        static Matrix4x4f ComputeHeadForwardKinematics(float pan, float tilt);

    public:
        static const float CAMERA_DISTANCE; //mm
        static const float EYE_TILT_OFFSET_ANGLE; //degree
        static const float LEG_SIDE_OFFSET; //mm
        static const float THIGH_LENGTH; //mm
        static const float CALF_LENGTH; //mm
        static const float ANKLE_LENGTH; //mm
        static const float LEG_LENGTH; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

        static const float CW_LIMIT_R_SHOULDER_ROLL; // degree
        static const float CCW_LIMIT_R_SHOULDER_ROLL; // degree
        static const float CW_LIMIT_L_SHOULDER_ROLL; // degree
        static const float CCW_LIMIT_L_SHOULDER_ROLL; // degree
        static const float CW_LIMIT_R_ELBOW; // degree
        static const float CCW_LIMIT_R_ELBOW; // degree
        static const float CW_LIMIT_L_ELBOW; // degree
        static const float CCW_LIMIT_L_ELBOW; // degree
        static const float CW_LIMIT_R_HIP_YAW; // degree
        static const float CCW_LIMIT_R_HIP_YAW; // degree
        static const float CW_LIMIT_L_HIP_YAW; // degree
        static const float CCW_LIMIT_L_HIP_YAW; // degree
        static const float CW_LIMIT_R_HIP_ROLL; // degree
        static const float CCW_LIMIT_R_HIP_ROLL; // degree
        static const float CW_LIMIT_L_HIP_ROLL; // degree
        static const float CCW_LIMIT_L_HIP_ROLL; // degree
        static const float CW_LIMIT_R_HIP_PITCH; // degree
        static const float CCW_LIMIT_R_HIP_PITCH; // degree
        static const float CW_LIMIT_L_HIP_PITCH; // degree
        static const float CCW_LIMIT_L_HIP_PITCH; // degree
        static const float CW_LIMIT_R_KNEE; // degree
        static const float CCW_LIMIT_R_KNEE; // degree
        static const float CW_LIMIT_L_KNEE; // degree
        static const float CCW_LIMIT_L_KNEE; // degree
        static const float CW_LIMIT_R_ANKLE_PITCH; // degree
        static const float CCW_LIMIT_R_ANKLE_PITCH; // degree
        static const float CW_LIMIT_L_ANKLE_PITCH; // degree
        static const float CCW_LIMIT_L_ANKLE_PITCH; // degree
        static const float CW_LIMIT_R_ANKLE_ROLL; // degree
        static const float CCW_LIMIT_R_ANKLE_ROLL; // degree
        static const float CW_LIMIT_L_ANKLE_ROLL; // degree
        static const float CCW_LIMIT_L_ANKLE_ROLL; // degree
        static const float CW_LIMIT_HEAD_PAN; // degree
        static const float CCW_LIMIT_HEAD_PAN; // degree
        static const float CW_LIMIT_HEAD_TILT; // degree
        static const float CCW_LIMIT_HEAD_TILT; // degree

        ~Kinematics();

    };
}

#endif
