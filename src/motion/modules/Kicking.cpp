/**
 *  @autor arssivka
 *  @date 9/20/17
 */

#include <motion/modules/Kicking.h>


void Robot::Kicking::Kick(int leg, float leg_x_offset, float leg_y_offset, float leg_z_offset, float leg_yaw_offset,
                          float leg_target_x_offset, float leg_target_y_offset) {
    if (m_kicking_done) {
        m_time = 0;
        m_kicking_done = false;

        m_kicking_leg = leg;
        m_kick_target_x_offset = leg_target_x_offset;
        m_kick_target_y_offset = leg_target_y_offset;
        m_kick_x_offset = leg_x_offset;
        m_kick_y_offset = leg_y_offset;
        m_kick_z_offset = leg_z_offset;
        m_kick_yaw_offset = leg_yaw_offset;
    }
}

bool Robot::Kicking::IsDone() const noexcept {
    return m_kicking_done;
}

void Robot::Kicking::Initialize() {

}

void Robot::Kicking::Process() {
    if (m_kicking_done) return;

    const float TIME_UNIT = MotionModule::TIME_UNIT;
    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    const int dir[14] = {-1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1};
    const float initAngle[14] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -48.345f, 41.313f};
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
    float b_x = 0;
    float b_y = 0;
    float b_z = 0;
    float b_a = 0;
    float b_b = 0;
    float b_c = 0;

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
            leg_x_active = wsin(local_time, period, 0.0, m_kick_x_offset, 0.0);
            leg_y_active = wsin(local_time, period, 0.0, m_kick_y_offset, 0.0);
            leg_z_active = wsin(local_time, period, 0.0, m_kick_z_offset, 0.0);
            leg_a_active = 0;
            leg_b_active = 0;
            leg_c_active = wsin(local_time, period, 0.0, m_kick_yaw_offset, 0.0);

            b_x = wsin(local_time, period, 0.0, m_cur_body_x_offset, 0.0);
            b_y = wsin(local_time, period, 0.0, m_cur_body_y_offset, 0.0);
            b_z = wsin(local_time, period, 0.0, m_cur_body_z_offset, 0.0);
            b_a = wsin(local_time, period, 0.0, m_cur_body_roll_offset, 0.0);
            b_b = m_cur_body_roll_offset;
            b_c = 0.0;
            break;
        case (PHASE_KICKING):
            period = m_cur_kicking_duration * 2.0f;
            local_time = m_time - m_cur_shifting_body_duration;
            leg_x_active = wsin(local_time, period, 0.0, m_kick_x_offset - m_kick_target_x_offset, m_kick_x_offset);
            leg_y_active = wsin(local_time, period, 0.0, m_kick_y_offset - m_kick_target_y_offset, m_kick_y_offset);
            leg_z_active = m_kick_z_offset;
            leg_a_active = 0;
            leg_b_active = 0;
            leg_c_active = m_kick_yaw_offset;

            b_x = m_cur_body_x_offset;
            b_y = m_cur_body_y_offset;
            b_z = m_cur_body_z_offset;
            b_a = m_cur_body_roll_offset;
            b_b = m_cur_body_roll_offset;
            b_c = 0.0;
            break;
        case (PHASE_RESTORING):
            period = m_cur_shifting_body_duration * 4.0f;
            local_time = m_time - m_cur_kicking_duration - m_shifting_body_duration;
            leg_x_active = wsin(local_time, period, m_shifting_body_duration, m_kick_x_offset, 0.0);
            leg_y_active = wsin(local_time, period, m_shifting_body_duration, m_kick_y_offset, 0.0);
            leg_z_active = wsin(local_time, period, m_shifting_body_duration, m_kick_z_offset, 0.0);
            leg_a_active = 0;
            leg_b_active = 0;
            leg_c_active = wsin(local_time, period, m_shifting_body_duration, m_kick_yaw_offset, 0.0);

            b_x = wsin(local_time, period, m_shifting_body_duration, m_cur_body_x_offset, 0.0);
            b_y = wsin(local_time, period, m_shifting_body_duration, m_cur_body_y_offset, 0.0);
            b_z = wsin(local_time, period, m_shifting_body_duration, m_cur_body_z_offset, 0.0);
            b_a = wsin(local_time, period, m_shifting_body_duration, m_cur_body_roll_offset, 0.0);
            b_b = m_cur_body_roll_offset;
            b_c = 0.0;
            break;
        case (PHASE_DONE):
        default:
            m_kicking_done = true;
            m_time = 0.0;
            return;
    }

    leg_x_active -= b_x;
    leg_y_active -= b_y;
    leg_z_active -= b_z;
    leg_a_active -= b_a;
    leg_b_active -= b_b;
    leg_c_active -= b_c;

    leg_x_support -= b_x;
    leg_y_support -= b_y;
    leg_z_support -= b_z;
    leg_a_support -= b_a;
    leg_b_support -= b_b;
    leg_c_support -= b_c;

    float* support_leg = l_joints;
    float* active_leg = r_joints;
    if (m_kicking_leg == LEFT_FEG) {
        std::swap(support_leg, active_leg);
    }

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

    for (int i = 0; i < 6; ++i) out_value[i] = (int) (r_joints[i] * MX28::RATIO_RADIANS2VALUE);
    for (int i = 0; i < 6; ++i) out_value[i + 6] = (int) (r_joints[i] * MX28::RATIO_RADIANS2VALUE);

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
    m_Joint.SetValue(JointData::ID_R_SHOULDER_PITCH, out_value[12]); // TODO Shoulders
    m_Joint.SetValue(JointData::ID_L_SHOULDER_PITCH, out_value[13]); // TODO !!!!!!!!!
    m_Joint.SetAngle(JointData::ID_HEAD_PAN, 0.0f); // TODO Head

    m_time += TIME_UNIT;
}

void Robot::Kicking::Break() {
    m_kicking_done = true;
}

void Robot::Kicking::UpdateTimeParameters() {
    if (m_kicking_done) {
        m_phase = PHASE_DONE;
        m_time = 0;
    } else {
        float sum = 0;

        sum += m_shifting_body_duration;
        if (m_time < sum) {
            m_phase = PHASE_SHIFTING_BODY;
            return;
        }

        sum += m_kicking_duration;
        if (m_time < sum) {
            m_phase = PHASE_KICKING;
            return;
        }

        sum += m_restoring_duration;
        if (m_time < sum) {
            m_phase = PHASE_RESTORING;
            return;
        }

        m_phase = PHASE_DONE;
    }
}

void Robot::Kicking::UpdateActiveParams() {
    m_cur_shifting_body_duration = m_shifting_body_duration;
    m_cur_kicking_duration = m_kicking_duration;
    m_cur_restoring_duration_active = m_restoring_duration;

    m_cur_body_x_offset = m_body_x_offset;
    m_cur_body_y_offset = m_body_y_offset;
    m_cur_body_z_offset = m_body_z_offset;
    m_cur_body_pitch_offset = m_body_pitch_offset;
    m_cur_body_roll_offset = m_body_roll_offset;

    m_cur_arm_swing_gain = m_arm_swing_gain;
    m_cur_balance_roll_gain = m_balance_roll_gain;
    m_cur_balance_pitch_gain = m_balance_pitch_gain;
    m_cur_balance_enabled = m_balance_enabled;
}

Robot::Kicking* Robot::Kicking::GetInstance() {
    static Kicking instance;
    return &instance;
}

