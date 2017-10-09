/**
 *  @autor arssivka
 *  @date 9/20/17
 */

#include <motion/modules/Kicking.h>
#include <iostream>
#include <log/Logger.h>


void Robot::Kicking::Kick() {
    if (m_done) {
        m_time = 0;
        m_done = false;
    }
}

bool Robot::Kicking::IsRunning() const noexcept {
    return !m_done;
}

void Robot::Kicking::Initialize() {
    m_time = 0;
    m_done = true;
}

void Robot::Kicking::Process() {
    if (m_done) return;

    constexpr float TIME_UNIT = MotionModule::TIME_UNIT;
    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    constexpr int dir[14] = {-1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1};
    constexpr float initAngle[14] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -48.345f, 41.313f};
    int out_value[14] = {};
    // Left leg endpoint transformation
    float r_joints[6] = {}; // for inverse kinematics calculations
    float leg_x_active = 0;
    float leg_y_active = 0;
    float leg_z_active = 0;
    float leg_a_active = 0;
    float leg_b_active = 0;
    float leg_c_active = 0;
    // Right leg endpoint transformation
    float l_joints[6] = {}; // for inverse kinematics calculations;
    float leg_x_support = 0;
    float leg_y_support = 0;
    float leg_z_support = 0;
    float leg_a_support = 0;
    float leg_b_support = 0;
    float leg_c_support = 0;
    // Body transformation
    float body_x = 0;
    float body_y = 0;
    float body_z = 0;
    // Shoulder rotations
    float r_shoulder = 0;
    float l_shoulder = 0;

    if (m_time == 0) {
        UpdateActiveParams();
    }

    UpdateTimeParameters();

    // We calculates only right leg motion
    // Values will be mirrored if we kicks by left leg
    float period = 0.0;
    float local_time = 0.0;
    switch (m_phase) {
        case (PHASE_SHIFTING_BODY):
            period = m_cur_shifting_body_duration * 4.0f;
            local_time = m_time;
            leg_x_active = wsin(local_time, period, 0.0, m_cur_kick_x_offset, 0.0);
            leg_y_active = wsin(local_time, period, 0.0, m_cur_kick_y_offset, 0.0);
            leg_z_active = wsin(local_time, period, 0.0, m_cur_kick_z_offset, 0.0);
            leg_a_active = 0;
            leg_b_active = 0;
            leg_c_active = wsin(local_time, period, 0.0, m_cur_kick_yaw_offset, 0.0);

            body_x = wsin(local_time, period, 0.0, m_cur_body_x_offset - m_cur_body_init_x_offset, m_cur_body_init_x_offset);
            body_y = wsin(local_time, period, 0.0, m_cur_body_y_offset - m_cur_body_init_y_offset, m_cur_body_init_y_offset);
            body_z = wsin(local_time, period, 0.0, m_cur_body_z_offset - m_cur_body_init_z_offset, m_cur_body_init_z_offset);
            break;
        case (PHASE_KICKING):
            period = m_cur_kicking_duration * 2.0f;
            local_time = m_time - m_cur_shifting_body_duration;
            leg_x_active = wsin(local_time, period, 0.0, m_cur_kick_target_x_offset - m_cur_kick_x_offset, m_cur_kick_x_offset);
            leg_y_active = wsin(local_time, period, 0.0, m_cur_kick_target_y_offset - m_cur_kick_y_offset, m_cur_kick_y_offset);
            leg_z_active = m_cur_kick_z_offset;
            leg_a_active = 0;
            leg_b_active = 0;
            leg_c_active = m_cur_kick_yaw_offset;

            body_x = m_cur_body_x_offset;
            body_y = m_cur_body_y_offset;
            body_z = m_cur_body_z_offset;
            break;
        case (PHASE_RESTORING):
            period = m_cur_restoring_duration * 4.0f;
            local_time = m_time - m_cur_kicking_duration - m_cur_shifting_body_duration;
            leg_x_active = wsin(local_time, period, -half_pi, m_cur_kick_x_offset, 0.0);
            leg_y_active = wsin(local_time, period, -half_pi, m_cur_kick_y_offset, 0.0);
            leg_z_active = wsin(local_time, period, -half_pi, m_cur_kick_z_offset, 0.0);
            leg_a_active = 0;
            leg_b_active = 0;
            leg_c_active = wsin(local_time, period, -half_pi, m_cur_kick_yaw_offset, 0.0);

            body_x = wsin(local_time, period, -half_pi, m_cur_body_x_offset - m_cur_body_init_x_offset, m_cur_body_init_x_offset);
            body_y = wsin(local_time, period, -half_pi, m_cur_body_y_offset - m_cur_body_init_y_offset, m_cur_body_init_y_offset);
            body_z = wsin(local_time, period, -half_pi, m_cur_body_z_offset - m_cur_body_init_z_offset, m_cur_body_init_z_offset);
            break;
        case (PHASE_DONE):
        default:
            if (m_debug) {
                LOG_DEBUG << "KICKING: Kick was finished";
            }
            m_done = true;
            m_time = 0.0;
            return;
    }

    float* support_leg = l_joints;
    float* active_leg = r_joints;
    if (m_cur_kicking_leg == LEFT_FEG) {
        body_y = -body_y;
        std::swap(support_leg, active_leg);
        std::swap(r_shoulder, l_shoulder);

    }

    m_time += TIME_UNIT;

    // Shift leg position by body offset
    leg_x_active -= body_x;
    leg_y_active -= body_y;
    leg_z_active -= body_z;

    leg_x_support -= body_x;
    leg_y_support -= body_y;
    leg_z_support -= body_z;

    bool ik_status = true;
    ik_status = ik_status && Kinematics::ComputeLegInverseKinematics(
            active_leg,
            leg_x_active,
            leg_y_active,
            leg_z_active,
            leg_a_active,
            leg_b_active,
            leg_c_active
    );

    ik_status = ik_status && Kinematics::ComputeLegInverseKinematics(
            support_leg,
            leg_x_support,
            leg_y_support,
            leg_z_support,
            leg_a_support,
            leg_b_support,
            leg_c_support
    );

    if (!ik_status) {
        return;
    }

    // Add body pitch offset
    r_joints[1] += m_cur_body_init_pitch_offset * sinf(r_joints[0]);
    r_joints[2] += m_cur_body_init_pitch_offset * cosf(r_joints[0]);
    l_joints[1] += m_cur_body_init_pitch_offset * sinf(l_joints[0]);
    l_joints[2] += m_cur_body_init_pitch_offset * cosf(l_joints[0]);

    for (int i = 0; i < 6; ++i) {
        out_value[i] = (int) (r_joints[i] * MX28::RATIO_RADIANS2VALUE * dir[i]) +
                MX28::Angle2Value(initAngle[i]);
    }
    for (int i = 0; i < 6; ++i) {
        out_value[i + 6] = (int) (l_joints[i] * MX28::RATIO_RADIANS2VALUE * dir[i + 6]) +
                MX28::Angle2Value(initAngle[i + 6]);
    }

    m_Joint.SetValue(JointData::ID_R_HIP_YAW, out_value[0]);
    m_Joint.SetValue(JointData::ID_R_HIP_ROLL, out_value[1]);
    m_Joint.SetValue(JointData::ID_R_HIP_PITCH, out_value[2]);
    m_Joint.SetValue(JointData::ID_R_KNEE, out_value[3]);
    m_Joint.SetValue(JointData::ID_R_ANKLE_PITCH, out_value[4]);
    m_Joint.SetValue(JointData::ID_R_ANKLE_ROLL, out_value[5]);
    m_Joint.SetValue(JointData::ID_L_HIP_YAW, out_value[6]);
    m_Joint.SetValue(JointData::ID_L_HIP_ROLL, out_value[7]);
    m_Joint.SetValue(JointData::ID_L_HIP_PITCH, out_value[8]);
    m_Joint.SetValue(JointData::ID_L_KNEE, out_value[9]);
    m_Joint.SetValue(JointData::ID_L_ANKLE_PITCH, out_value[10]);
    m_Joint.SetValue(JointData::ID_L_ANKLE_ROLL, out_value[11]);
//    m_Joint.SetValue(JointData::ID_R_SHOULDER_PITCH, out_value[12]); // TODO Shoulders
//    m_Joint.SetValue(JointData::ID_L_SHOULDER_PITCH, out_value[13]); // TODO !!!!!!!!!
//    m_Joint.SetAngle(JointData::ID_HEAD_PAN, 0.0f); // TODO Head
}

void Robot::Kicking::Break() {
    if (m_debug) {
        LOG_DEBUG << "KICKING: Kick was interrupted";
    }
    m_done = true;
}

void Robot::Kicking::UpdateTimeParameters() {
    if (m_done) {
        m_phase = PHASE_DONE;
        m_time = 0;
    } else {
        float sum = 0;

        sum += m_cur_shifting_body_duration;
        if (m_time <= sum) {
            m_phase = PHASE_SHIFTING_BODY;
            return;
        }

        sum += m_cur_kicking_duration;
        if (m_time <= sum) {
            m_phase = PHASE_KICKING;
            return;
        }

        sum += m_cur_restoring_duration;
        if (m_time <= sum) {
            m_phase = PHASE_RESTORING;
            return;
        }

        m_phase = PHASE_DONE;
    }
}

void Robot::Kicking::UpdateActiveParams() noexcept {
    m_cur_kicking_leg = m_kicking_leg;
    m_cur_kick_target_x_offset = m_kick_target_x_offset;
    m_cur_kick_target_y_offset = m_kick_target_y_offset;
    m_cur_kick_x_offset = m_kick_x_offset;
    m_cur_kick_y_offset = m_kick_y_offset;
    m_cur_kick_z_offset = m_kick_z_offset;
    m_cur_kick_yaw_offset = m_kick_yaw_offset;

    m_cur_shifting_body_duration = m_shifting_body_duration;
    m_cur_kicking_duration = m_kicking_duration;
    m_cur_restoring_duration = m_restoring_duration;

    m_cur_body_init_x_offset = m_body_init_x_offset;
    m_cur_body_init_y_offset = m_body_init_y_offset;
    m_cur_body_init_z_offset = m_body_init_z_offset;
    m_cur_body_init_pitch_offset = m_body_init_pitch_offset;

    m_cur_body_x_offset = m_body_x_offset;
    m_cur_body_y_offset = m_body_y_offset;
    m_cur_body_z_offset = m_body_z_offset;

    m_cur_arm_swing_gain = m_arm_swing_gain;
    m_cur_balance_roll_gain = m_balance_roll_gain;
    m_cur_balance_pitch_gain = m_balance_pitch_gain;
    m_cur_balance_enabled = m_balance_enabled;

    if (m_debug) {
        LOG_DEBUG << "KICKING: kick was started";
    }
}

Robot::Kicking* Robot::Kicking::GetInstance() {
    static Kicking instance;
    return &instance;
}

int Robot::Kicking::GetKickingLeg() const noexcept {
    return m_kicking_leg;
}

void Robot::Kicking::SetKickingLeg(int kicking_leg) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: kicking_leg = " << (m_kicking_leg == RIGHT_LEG ? "RIGHT_LEG" : "LEFT_LEG");
    }
    m_kicking_leg = kicking_leg;
}

float Robot::Kicking::GetKickTargetXOffset() const noexcept {
    return m_kick_target_x_offset;
}

void Robot::Kicking::SetKickTargetXOffset(float kick_target_x_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: kicking_target_x_offset = " << kick_target_x_offset;
    }
    m_kick_target_x_offset = kick_target_x_offset;
}

float Robot::Kicking::GetKickTargetYOffset() const noexcept {
    return m_kick_target_y_offset;
}

void Robot::Kicking::SetKickTargetYOffset(float kick_target_y_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: kicking_target_y_offset = " << kick_target_y_offset;
    }
    m_kick_target_y_offset = kick_target_y_offset;
}

float Robot::Kicking::GetKickXOffset() const noexcept {
    return m_kick_x_offset;
}

void Robot::Kicking::SetKickXOffset(float kick_x_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: kicking_x_offset = " << kick_x_offset;
    }
    m_kick_x_offset = kick_x_offset;
}

float Robot::Kicking::GetKickYOffset() const noexcept {
    return m_kick_y_offset;
}

void Robot::Kicking::SetKickYOffset(float kick_y_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: kicking_y_offset = " << kick_y_offset;
    }
    m_kick_y_offset = kick_y_offset;
}

float Robot::Kicking::GetLickZOffset() const noexcept {
    return m_kick_z_offset;
}

void Robot::Kicking::SetKickZOffset(float kick_z_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: kicking_z_offset = " << kick_z_offset;
    }
    m_kick_z_offset = kick_z_offset;
}

float Robot::Kicking::GetKickYawOffset() const noexcept {
    return m_kick_yaw_offset;
}

void Robot::Kicking::SetKickYawOffset(float kick_yaw_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: kicking_yaw_offset = " << kick_yaw_offset;
    }
    m_kick_yaw_offset = kick_yaw_offset;
}

float Robot::Kicking::GetShiftingBodyDuration() const noexcept {
    return m_shifting_body_duration;
}

void Robot::Kicking::SetShiftingBodyDuration(float shifting_body_duration) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: shifting_body_duration = " << shifting_body_duration;
    }
    m_shifting_body_duration = shifting_body_duration;
}

float Robot::Kicking::GetKickingDuration() const noexcept {
    return m_kicking_duration;
}

void Robot::Kicking::SetKickingDuration(float kicking_duration) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: kicking_duration = " << kicking_duration;
    }
    m_kicking_duration = kicking_duration;
}

float Robot::Kicking::GetRestoringDuration() const noexcept {
    return m_restoring_duration;
}

void Robot::Kicking::SetRestoringDuration(float restoring_duration) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: restoring_duration = " << restoring_duration;
    }
    m_restoring_duration = restoring_duration;
}

float Robot::Kicking::GetBodyInitXOffset() const noexcept {
    return m_body_init_x_offset;
}

void Robot::Kicking::SetBodyInitXOffset(float body_init_x_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: body_init_x_offset = " << body_init_x_offset;
    }
    m_body_init_x_offset = body_init_x_offset;
}

float Robot::Kicking::GetBodyInitYOffset() const noexcept {
    return m_body_init_y_offset;
}

void Robot::Kicking::SetBodyInitYOffset(float body_init_y_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: body_init_y_offset = " << body_init_y_offset;
    }
    m_body_init_y_offset = body_init_y_offset;
}

float Robot::Kicking::GetBodyInitZOffset() const noexcept {
    return m_body_init_z_offset;
}

void Robot::Kicking::SetBodyInitZOffset(float body_init_z_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: body_init_z_offset = " << body_init_z_offset;
    }
    m_body_init_z_offset = body_init_z_offset;
}

float Robot::Kicking::GetBodyInitPitchOffset() const noexcept {
    return m_body_init_pitch_offset;
}

void Robot::Kicking::SetBodyInitPitchOffset(float body_init_pitch_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: body_init_pitch_offset = " << body_init_pitch_offset;
    }
    m_body_init_pitch_offset = body_init_pitch_offset;
}

float Robot::Kicking::GetBodyXOffset() const noexcept {
    return m_body_x_offset;
}

void Robot::Kicking::SetBodyXOffset(float body_x_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: body_x_offset = " << body_x_offset;
    }
    m_body_x_offset = body_x_offset;
}

float Robot::Kicking::GetBodyYOffset() const noexcept {
    return m_body_y_offset;
}

void Robot::Kicking::SetBodyYOffset(float body_y_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: body_y_offset = " << body_y_offset;
    }
    m_body_y_offset = body_y_offset;
}

float Robot::Kicking::GetBodyZOffset() const noexcept {
    return m_body_z_offset;
}

void Robot::Kicking::SetBodyZOffset(float body_z_offset) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: body_z_offset = " << body_z_offset;
    }
    m_body_z_offset = body_z_offset;
}

float Robot::Kicking::GetArmSwingGain() const noexcept {
    return m_arm_swing_gain;
}

void Robot::Kicking::SetArmSwingGain(float arm_swing_gain) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: arm_swing_gain = " << arm_swing_gain;
    }
    m_arm_swing_gain = arm_swing_gain;
}

float Robot::Kicking::GetBalanceRollGain() const noexcept {
    return m_balance_roll_gain;
}

void Robot::Kicking::SetBalanceRollGain(float balance_roll_gain) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: balance_roll_gain = " << balance_roll_gain;
    }
    m_balance_roll_gain = balance_roll_gain;
}

float Robot::Kicking::GetBalancePitchGain() const noexcept {
    return m_balance_pitch_gain;
}

void Robot::Kicking::SetBalancePitchGain(float balance_pitch_gain) noexcept {
    if (m_debug) {
        LOG_DEBUG << "KICKING: balance_pitch_gain = " << balance_pitch_gain;
    }
    m_balance_pitch_gain = balance_pitch_gain;
}

bool Robot::Kicking::GetBalanceEnabled() const noexcept {
    return m_balance_enabled;
}

void Robot::Kicking::SetBalanceEnabled(bool balance_enabled) noexcept {
    if (m_debug) {
        if (balance_enabled) {
            LOG_DEBUG << "KICKING: balance was enabled";
        } else {
            LOG_DEBUG << "KICKING: balance was disabled";
        }
    }
    m_balance_enabled = balance_enabled;
}

