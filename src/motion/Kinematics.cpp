/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <cmath>
#include "motion/Kinematics.h"

using namespace Robot;

const float Kinematics::CAMERA_DISTANCE = 33.2; //mm
const float Kinematics::EYE_TILT_OFFSET_ANGLE = 40.0; //degree
const float Kinematics::LEG_SIDE_OFFSET = 37.0; //mm
const float Kinematics::THIGH_LENGTH = 93.0; //mm
const float Kinematics::CALF_LENGTH = 93.0; //mm
const float Kinematics::ANKLE_LENGTH = 33.5; //mm
const float Kinematics::LEG_LENGTH = 219.5; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

const float Kinematics::CW_LIMIT_R_SHOULDER_ROLL = -75.0; // degree
const float Kinematics::CCW_LIMIT_R_SHOULDER_ROLL = 135.0; // degree
const float Kinematics::CW_LIMIT_L_SHOULDER_ROLL = -135.0; // degree
const float Kinematics::CCW_LIMIT_L_SHOULDER_ROLL = 75.0; // degree
const float Kinematics::CW_LIMIT_R_ELBOW = -95.0; // degree
const float Kinematics::CCW_LIMIT_R_ELBOW = 70.0; // degree
const float Kinematics::CW_LIMIT_L_ELBOW = -70.0; // degree
const float Kinematics::CCW_LIMIT_L_ELBOW = 95.0; // degree
const float Kinematics::CW_LIMIT_R_HIP_YAW = -123.0; // degree
const float Kinematics::CCW_LIMIT_R_HIP_YAW = 53.0; // degree
const float Kinematics::CW_LIMIT_L_HIP_YAW = -53.0; // degree
const float Kinematics::CCW_LIMIT_L_HIP_YAW = 123.0; // degree
const float Kinematics::CW_LIMIT_R_HIP_ROLL = -45.0; // degree
const float Kinematics::CCW_LIMIT_R_HIP_ROLL = 59.0; // degree
const float Kinematics::CW_LIMIT_L_HIP_ROLL = -59.0; // degree
const float Kinematics::CCW_LIMIT_L_HIP_ROLL = 45.0; // degree
const float Kinematics::CW_LIMIT_R_HIP_PITCH = -100.0; // degree
const float Kinematics::CCW_LIMIT_R_HIP_PITCH = 29.0; // degree
const float Kinematics::CW_LIMIT_L_HIP_PITCH = -29.0; // degree
const float Kinematics::CCW_LIMIT_L_HIP_PITCH = 100.0; // degree
const float Kinematics::CW_LIMIT_R_KNEE = -6.0; // degree
const float Kinematics::CCW_LIMIT_R_KNEE = 130.0; // degree
const float Kinematics::CW_LIMIT_L_KNEE = -130.0; // degree
const float Kinematics::CCW_LIMIT_L_KNEE = 6.0; // degree
const float Kinematics::CW_LIMIT_R_ANKLE_PITCH = -72.0; // degree
const float Kinematics::CCW_LIMIT_R_ANKLE_PITCH = 80.0; // degree
const float Kinematics::CW_LIMIT_L_ANKLE_PITCH = -80.0; // degree
const float Kinematics::CCW_LIMIT_L_ANKLE_PITCH = 72.0; // degree
const float Kinematics::CW_LIMIT_R_ANKLE_ROLL = -44.0; // degree
const float Kinematics::CCW_LIMIT_R_ANKLE_ROLL = 63.0; // degree
const float Kinematics::CW_LIMIT_L_ANKLE_ROLL = -63.0; // degree
const float Kinematics::CCW_LIMIT_L_ANKLE_ROLL = 44.0; // degree
const float Kinematics::CW_LIMIT_HEAD_PAN = -90.0; // degree
const float Kinematics::CCW_LIMIT_HEAD_PAN = 90.0; // degree
const float Kinematics::CW_LIMIT_HEAD_TILT = -25.0; // degree
const float Kinematics::CCW_LIMIT_HEAD_TILT = 55.0; // degree

Kinematics::Kinematics() {}

bool Kinematics::ComputeLegInverseKinematics(float* out, float x, float y, float z, float a, float b, float c) {
    Matrix3D Tad, Tda, Tcd, Tdc, Tac;
    Vector3D vec;
    float _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
    float LEG_LENGTH = Kinematics::LEG_LENGTH;
    float THIGH_LENGTH = Kinematics::THIGH_LENGTH;
    float CALF_LENGTH = Kinematics::CALF_LENGTH;
    float ANKLE_LENGTH = Kinematics::ANKLE_LENGTH;

    Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH), Vector3D(a * 180.0 / M_PI, b * 180.0 / M_PI, c * 180.0 / M_PI));

    vec.X = x + Tad.m[2] * ANKLE_LENGTH;
    vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
    vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;

    // Get Knee
    _Rac = vec.Length();
    _Acos = acos(
            (_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
    if (std::isnan(_Acos) == 1)
        return false;
    *(out + 3) = _Acos;

    // Get Ankle Roll
    Tda = Tad;
    if (Tda.Inverse() == false)
        return false;
    _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
    _l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
    if (_m > 1.0)
        _m = 1.0;
    else if (_m < -1.0)
        _m = -1.0;
    _Acos = acos(_m);
    if (std::isnan(_Acos) == 1)
        return false;
    if (Tda.m[7] < 0.0)
        *(out + 5) = -_Acos;
    else
        *(out + 5) = _Acos;

    // Get Hip Yaw
    Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(*(out + 5) * 180.0 / M_PI, 0, 0));
    Tdc = Tcd;
    if (!Tdc.Inverse())
        return false;
    Tac = Tad * Tdc;
    _Atan = atan2(-Tac.m[1], Tac.m[5]);
    if (std::isinf(_Atan) == 1)
        return false;
    *(out) = _Atan;

    // Get Hip Roll
    _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
    if (std::isinf(_Atan) == 1)
        return false;
    *(out + 1) = _Atan;

    // Get Hip Pitch and Ankle Pitch
    _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
    if (std::isinf(_Atan) == 1)
        return false;
    _theta = _Atan;
    _k = sin(*(out + 3)) * CALF_LENGTH;
    _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
    _m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
    _n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
    _c = (_n - _k * _s) / _l;
    _Atan = atan2(_s, _c);
    if (std::isinf(_Atan) == 1)
        return false;
    *(out + 2) = _Atan;
    *(out + 4) = _theta - *(out + 3) - *(out + 2);

    return true;
}


Kinematics::~Kinematics() {}

Matrix4x4f Kinematics::ComputeLegForwardKinematics(float pelvis, float tight_roll, float tight_pitch,
                                                   float knee_pitch, float ankle_pitch, float ankle_roll) {
    const float s1 = sinf(pelvis);
    const float c1 = cosf(pelvis);
    const float s2 = sinf(tight_roll);
    const float c2 = cosf(tight_roll);
    const float s3 = sinf(tight_pitch);
    const float c3 = cosf(tight_pitch);
    const float s4 = sinf(knee_pitch);
    const float c4 = cosf(knee_pitch);
    const float s5 = sinf(ankle_pitch);
    const float c5 = cosf(ankle_pitch);
    const float s6 = sinf(ankle_roll);
    const float c6 = cosf(ankle_roll);

    const float c1s2 = c1 * s2;
    const float s1c3 = s1 * c3;
    const float s1s3 = s1 * s3;
    const float s2c3 = s2 * c3;
    const float s2s3 = s2 * s3;
    const float c4c5 = c4 * c5;
    const float c4s5 = c4 * s5;
    const float s4s5 = s4 * s5;
    const float s4c5 = s4 * c5;
    const float c1c2c3 = c1 * c2 * c3;
    const float s3c1c2 = c1 * c2 * s3;

    const float r11 = (c1c2c3 - s1s3) * (c4c5 - s4s5) * c6 + (-s3c1c2 - s1c3) * (s4c5 + c4s5) * c6 - c1s2 * s6;
    const float r12 = (c1c2c3 - s1s3) * (-c4c5 + s4s5) * s6 + (-s3c1c2 - s1c3) * (-s4c5 - c4s5) * s6 - c1s2 * c6;
    const float r13 = (c1c2c3 - s1s3) * (c4s5 + s4c5) + (-s3c1c2 - s1c3) * (s4s5 - c4c5);
    const float r31 = (s2c3 * c6) * (c4c5 - s4s5) - s2s3 * (s4c5 + c4s5) * c6 + c2 * s6;
    const float r32 = (s2c3 * s6) * (-c4c5 + s4s5) - s2s3 * (-s4c5 - c4s5) * s6 + c2 * c6;
    const float r33 = s2c3 * (c4s5 + s4c5) - s2s3 * (s4s5 - c4c5);
    const float r21 = r12 * r33 - r13 * r32;
    const float r22 = r13 * r31 - r11 * r33;
    const float r23 = r11 * r32 - r12 * r31;

    const float px = (c1c2c3 - s1s3) * (c4 * CALF_LENGTH + THIGH_LENGTH) +
                     (-s3c1c2 - s1c3) * (s4 * CALF_LENGTH) + r11 * ANKLE_LENGTH;
    const float py = (s1 * c2 * c3 + c1 * s3) * (c4 * CALF_LENGTH + THIGH_LENGTH) +
                     (-s3 * s1 * c2 + c1 * c3) * (s4 * CALF_LENGTH) + r12 * ANKLE_LENGTH;
    const float pz = s2c3 * (c4 * CALF_LENGTH + THIGH_LENGTH) -
                     s2s3 * (s4 * CALF_LENGTH) + r13 * ANKLE_LENGTH;

    Matrix4x4f res;
    res << r11, r12, r13, px,
            r21, r22, r23, py,
            r31, r32, r33, pz,
            0.0, 0.0, 0.0, 1.0;
    return res;
}
