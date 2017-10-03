/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <cmath>
#include <math/AngleTools.h>
#include <iostream>
#include "motion/Kinematics.h"

using namespace Robot;

bool Kinematics::ComputeLegInverseKinematics(float* out, float x, float y, float z, float a, float b, float c) {
    Matrix3D Tad, Tda, Tcd, Tdc, Tac;
    Vector3D vec;
    float _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
    float LEG_LENGTH = Kinematics::LEG_LENGTH;
    float THIGH_LENGTH = Kinematics::THIGH_LENGTH;
    float CALF_LENGTH = Kinematics::CALF_LENGTH;
    float ANKLE_LENGTH = Kinematics::ANKLE_LENGTH;

    Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH), Vector3D(degrees(a), degrees(b), degrees(c)));

    vec.X = x + Tad.m[2] * ANKLE_LENGTH;
    vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
    vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;

    // Get Knee
    _Rac = vec.Length();
    _Acos = acosf((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) /
                    (2 * THIGH_LENGTH * CALF_LENGTH));
    if (std::isnan(_Acos))
        return false;
    out[3] = _Acos;

    // Get Ankle Roll
    Tda = Tad;
    if (!Tda.Inverse())
        return false;
    _k = sqrtf(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
    _l = sqrtf(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
    if (_m > 1.0)
        _m = 1.0;
    else if (_m < -1.0)
        _m = -1.0f;
    _Acos = acosf(_m);
    if (std::isnan(_Acos))
        return false;
    if (Tda.m[7] < 0.0)
        out[5] = -_Acos;
    else
        out[5] = _Acos;

    // Get Hip Yaw
    Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(degrees(out[5]), 0, 0));
    Tdc = Tcd;
    if (!Tdc.Inverse())
        return false;
    Tac = Tad * Tdc;
    _Atan = atan2f(-Tac.m[1], Tac.m[5]);
    if (std::isinf(_Atan))
        return false;
    out[0] = _Atan;

    // Get Hip Roll
    _Atan = atan2f(Tac.m[9], -Tac.m[1] * sinf(out[0]) + Tac.m[5] * cosf(out[0]));
    if (std::isinf(_Atan))
        return false;
    out[1] = _Atan; //TODO: X2?

    // Get Hip Pitch and Ankle Pitch
    _Atan = atan2f(Tac.m[2] * cosf(out[0]) + Tac.m[6] * sinf(out[0]), Tac.m[0] * cosf(out[0]) + Tac.m[4] * sinf(out[0]));
    if (std::isinf(_Atan))
        return false;
    _theta = _Atan;
    _k = sinf(out[3]) * CALF_LENGTH;
    _l = -THIGH_LENGTH - cosf(out[3]) * CALF_LENGTH;
    _m = cosf(out[0]) * vec.X + sinf(out[0]) * vec.Y;
    _n = cosf(out[1]) * vec.Z + sinf(out[0]) * sinf(out[1]) * vec.X - cosf(out[0]) * sinf(out[1]) * vec.Y;
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
    _c = (_n - _k * _s) / _l;
    _Atan = atan2f(_s, _c);
    if (std::isinf(_Atan))
        return false;
    out[2] = _Atan;
    out[4] = _theta - out[3] - out[2];

    return true;
}


Kinematics::~Kinematics() {}

void Kinematics::ComputeLegForwardKinematics(Matrix4x4f& out, float pelvis, float tight_roll, float tight_pitch,
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
    const float r33 = (s2c3 * s6) * (-c4c5 + s4s5) - s2s3 * (-s4c5 - c4s5) * s6 + c2 * c6;
    const float r32 = s2c3 * (c4s5 + s4c5) - s2s3 * (s4s5 - c4c5);
    const float r21 = r12 * r33 - r13 * r32;
    const float r22 = r13 * r31 - r11 * r33;
    const float r23 = r11 * r32 - r12 * r31;

    const float px = s2c3 * (c4 * CALF_LENGTH + THIGH_LENGTH) -
                     s2s3 * (s4 * CALF_LENGTH);
    const float py = (s1 * c2 * c3 + c1 * s3) * (c4 * CALF_LENGTH + THIGH_LENGTH) +
                     (-s3 * s1 * c2 + c1 * c3) * (s4 * CALF_LENGTH); //TODO:: X2?
    const float pz = (c1c2c3 - s1s3) * (c4 * CALF_LENGTH + THIGH_LENGTH) +
                     (-s3c1c2 - s1c3) * (s4 * CALF_LENGTH);


            out << r11, r12, r13, px,
            r21, r22, r23, py,
            r31, r32, r33, pz,
            0.0, 0.0, 0.0, 1.0;

    //Rotates axises to the robot notation and apply ankle_length
    Matrix4x4f transform;
    transform << 0, -1, 0, 0,
            1, 0, 0, 0,
            0, 0, -1, -ANKLE_LENGTH,
            0, 0, 0, 1;
    out = transform * out;
}

void Kinematics::ComputeHeadForwardKinematics(Matrix4x4f& out, float pan, float tilt) {
//    // todo check order of angles
    const float s1 = sinf(tilt);
    const float c1 = cosf(tilt);
    const float s2 = sinf(pan);
    const float c2 = cosf(pan);

    const float r11 = c1 * c2;
    const float r12 = -c1 * s2; //TODO: Check the minus
    const float r13 = s1;

    const float r21 = s2;
    const float r22 = c2;
    const float r23 = 0.0f;

    const float r31 = s1 * c2;
    const float r32 = -s1 * s2; //TODO: Check the minus
    const float r33 = c1;

    const float px = r11 * CAMERA_OFFSET_X + r13 * CAMERA_OFFSET_Z;
    const float py = r21 * CAMERA_OFFSET_X + r23 * CAMERA_OFFSET_Z;
    const float pz = r31 * CAMERA_OFFSET_X + r33 * CAMERA_OFFSET_Z;

    out << r11, r12, r13, px,
           r21, r22, r23, py,
           r31, r32, r33, pz,
           0.0, 0.0, 0.0, 1.0;
}
