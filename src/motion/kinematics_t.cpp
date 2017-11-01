/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <cmath>
#include <math/angle_tools.h>
#include <iostream>
#include "motion/kinematics_t.h"

using namespace drwn;

bool kinematics_t::compute_leg_inverse_kinematics(float* out, float x, float y, float z, float a, float b, float c) {
    const float cyaw = cosf(c);
    const float syaw = sinf(c);
    const float cpitch = cosf(b);
    const float spitch = sinf(b);
    const float croll = cosf(a);
    const float sroll = sinf(a);

    // Apply roll
    // TODO Applying roll may cause wrong calculations. Check it.
    x += -sroll * spitch * ANKLE_LENGTH;
    y += sroll *          ANKLE_LENGTH;
    z += sroll * cpitch * ANKLE_LENGTH;
    // Apply pitch
    z += cpitch * ANKLE_LENGTH;
    x += -spitch * ANKLE_LENGTH;
    // Apply yaw
    const float oldx = x;
    const float oldy  = y;
    x = cyaw * oldx + syaw * oldy;
    y = cyaw * oldy - syaw * oldx;
    // Ankle offset
    z = LEG_LENGTH - z;
    const float offset_sqr = z * z + x * x;
    const float offset_dist = sqrtf(offset_sqr);

    if (offset_dist > CALF_LENGTH + THIGH_LENGTH) {
        return false;
    }

    // Hip yaw
    out[0] = c;
    // Hip roll
    out[1] = atan2f(y, z);
    // Hip pitch
    out[2] = -atan2f(x, z) -
            acosf((offset_sqr + THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) /
                          (2.0f * THIGH_LENGTH * offset_dist));
    // Knee pitch
    out[3] = pi - acosf((THIGH_LENGTH * THIGH_LENGTH + CALF_LENGTH * CALF_LENGTH - offset_sqr) /
                              (2.0f * THIGH_LENGTH * CALF_LENGTH));
    // Ankle pitch
    out[4] = -out[3] - out[2] - b;
    // Ankle roll
    out[5] = -out[1] + a;
    return true;
}

void kinematics_t::compute_leg_forward_kinematics(matrix_4x4f_t& out, float pelvis, float tight_roll, float tight_pitch,
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

    const float px = s2c3 * (c4 * CALF_LENGTH + THIGH_LENGTH) -
                     s2s3 * (s4 * CALF_LENGTH) + r13 * ANKLE_LENGTH;
    const float py = (c1c2c3 - s1s3) * (c4 * CALF_LENGTH + THIGH_LENGTH) +
                     (-s3c1c2 - s1c3) * (s4 * CALF_LENGTH) + r11 * ANKLE_LENGTH;
    const float pz = (s1 * c2 * c3 + c1 * s3) * (c4 * CALF_LENGTH + THIGH_LENGTH) +
                     (-s3 * s1 * c2 + c1 * c3) * (s4 * CALF_LENGTH) + r12 * ANKLE_LENGTH;

    out << r11, r12, r13, px,
           r21, r22, r23, py,
           r31, r32, r33, pz,
           0.0, 0.0, 0.0, 1.0;
}

void kinematics_t::compute_head_forward_kinematics(matrix_4x4f_t& out, float pan, float tilt) {
    // todo check order of angles
    const float s1 = sinf(pan);
    const float c1 = cosf(pan);
    const float s2 = sinf(tilt);
    const float c2 = cosf(tilt);

    const float r11 = c1 * c2;
    const float r12 = -c1 * s2;
    const float r13 = -s1;

    const float r21 = -s2;
    const float r22 = -c2;
    const float r23 = 0.0f;

    const float r31 = s1 * c2;
    const float r32 = -s1 * s2;
    const float r33 = c1;

    const float px = r11 * CAMERA_OFFSET_X + r13 * CAMERA_OFFSET_Z;
    const float py = r21 * CAMERA_OFFSET_X + r23 * CAMERA_OFFSET_Z;
    const float pz = r31 * CAMERA_OFFSET_X + r33 * CAMERA_OFFSET_Z;

    out << r11, r12, r13, px,
           r21, r22, r23, py,
           r31, r32, r33, pz,
           0.0, 0.0, 0.0, 1.0;
}
