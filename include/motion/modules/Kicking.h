/**
 *  @autor arssivka
 *  @date 9/20/17
 */

#pragma once


#include <utility>
#include <motion/MotionModule.h>
#include <math/AngleTools.h>
#include <motion/Kinematics.h>
#include <MX28.h>

namespace Robot {
    class Kicking
            : public MotionModule {
    public:
        enum {
            PHASE_DONE, // Kick are done
            PHASE_SHIFTING_BODY, // Shift center mass
            PHASE_KICKING, // Kicking
            PHASE_RESTORING  // Go back
        };

        enum {
            RIGHT_LEG,
            LEFT_FEG
        };

    public:
        Kicking() {}

        void Kick(int leg, float leg_x_offset, float leg_y_offset,
                  float leg_z_offset, float leg_yaw_offset,
                  float leg_target_x_offset, float leg_target_y_offset) {
            if (m_KickingDone) {
                m_Time = 0;
                m_KickingDone = false;

                m_KickingLeg = leg;
                m_KickTargetXOffset = leg_target_x_offset;
                m_KickTargetYOffset = leg_target_y_offset;
                m_KickXOffset = leg_x_offset;
                m_KickYOffset = leg_y_offset;
                m_KickZOffset = leg_z_offset;
                m_KickYawOffset = leg_yaw_offset;
            }
        }

        bool IsDone() const noexcept {
            return m_KickingDone;
        }

        void Initialize() override {

        }

        void Process() override {
            if (m_KickingDone) return;

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

            if (m_Time == 0) {
                UpdateActiveParams();
            }

            UpdateTimeParameters();

            // We calculates only right leg motion
            // Values will be mirrored if we kicks by left leg
            float period = 0.0;
            float local_time = 0.0;
            switch (m_Phase) {
                case (PHASE_SHIFTING_BODY):
                    period = m_ShiftingBodyDurationActive * 4.0f;
                    local_time = m_Time;
                    leg_x_active = wsin(local_time, period, 0.0, m_KickXOffset, 0.0);
                    leg_y_active = wsin(local_time, period, 0.0, m_KickYOffset, 0.0);
                    leg_z_active = wsin(local_time, period, 0.0, m_KickZOffset, 0.0);
                    leg_a_active = 0;
                    leg_b_active = 0;
                    leg_c_active = wsin(local_time, period, 0.0, m_KickYawOffset, 0.0);

                    b_x = wsin(local_time, period, 0.0, m_BodyXOffsetActive, 0.0);
                    b_y = wsin(local_time, period, 0.0, m_BodyYOffsetActive, 0.0);
                    b_z = wsin(local_time, period, 0.0, m_BodyZOffsetActive, 0.0);
                    b_a = wsin(local_time, period, 0.0, m_BodyRollOffsetActive, 0.0);
                    b_b = m_BodyRollOffsetActive;
                    b_c = 0.0;
                    break;
                case (PHASE_KICKING):
                    period = m_KickingDurationActive * 2.0f;
                    local_time = m_Time - m_ShiftingBodyDurationActive;
                    leg_x_active = wsin(local_time, period, 0.0, m_KickXOffset - m_KickTargetXOffset, m_KickXOffset);
                    leg_y_active = wsin(local_time, period, 0.0, m_KickYOffset - m_KickTargetYOffset, m_KickYOffset);
                    leg_z_active = m_KickZOffset;
                    leg_a_active = 0;
                    leg_b_active = 0;
                    leg_c_active = m_KickYawOffset;

                    b_x = m_BodyXOffsetActive;
                    b_y = m_BodyYOffsetActive;
                    b_z = m_BodyZOffsetActive;
                    b_a = m_BodyRollOffsetActive;
                    b_b = m_BodyRollOffsetActive;
                    b_c = 0.0;
                    break;
                case (PHASE_RESTORING):
                    period = m_ShiftingBodyDurationActive * 4.0f;
                    local_time = m_Time - m_KickingDurationActive - m_ShiftingBodyDuration;
                    leg_x_active = wsin(local_time, period, m_ShiftingBodyDuration, m_KickXOffset, 0.0);
                    leg_y_active = wsin(local_time, period, m_ShiftingBodyDuration, m_KickYOffset, 0.0);
                    leg_z_active = wsin(local_time, period, m_ShiftingBodyDuration, m_KickZOffset, 0.0);
                    leg_a_active = 0;
                    leg_b_active = 0;
                    leg_c_active = wsin(local_time, period, m_ShiftingBodyDuration, m_KickYawOffset, 0.0);

                    b_x = wsin(local_time, period, m_ShiftingBodyDuration, m_BodyXOffsetActive, 0.0);
                    b_y = wsin(local_time, period, m_ShiftingBodyDuration, m_BodyYOffsetActive, 0.0);
                    b_z = wsin(local_time, period, m_ShiftingBodyDuration, m_BodyZOffsetActive, 0.0);
                    b_a = wsin(local_time, period, m_ShiftingBodyDuration, m_BodyRollOffsetActive, 0.0);
                    b_b = m_BodyRollOffsetActive;
                    b_c = 0.0;
                    break;
                case (PHASE_DONE):
                default:
                    m_KickingDone = true;
                    m_Time = 0.0;
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
            if (m_KickingLeg == LEFT_FEG) {
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

            m_Time += TIME_UNIT;
        }

        void Break() {
            m_KickingDone = true;
        }

    private:
        void UpdateTimeParameters() {
            if (m_KickingDone) {
                m_Phase = PHASE_DONE;
                m_Time = 0;
            } else {
                float sum = 0;

                sum += m_ShiftingBodyDuration;
                if (m_Time < sum) {
                    m_Phase = PHASE_SHIFTING_BODY;
                    return;
                }

                sum += m_KickingDuration;
                if (m_Time < sum) {
                    m_Phase = PHASE_KICKING;
                    return;
                }

                sum += m_RestoringDuration;
                if (m_Time < sum) {
                    m_Phase = PHASE_RESTORING;
                    return;
                }

                m_Phase = PHASE_DONE;
            }
        }

        void UpdateActiveParams() {
            m_ShiftingBodyDurationActive = m_ShiftingBodyDuration;
            m_KickingDurationActive = m_KickingDuration;
            m_RestoringDurationActive = m_RestoringDuration;

            m_BodyXOffsetActive = m_BodyXOffset;
            m_BodyYOffsetActive = m_BodyYOffset;
            m_BodyZOffsetActive = m_BodyZOffset;
            m_BodyPitchOffsetActive = m_BodyPitchOffset;
            m_BodyRollOffsetActive = m_BodyRollOffset;

            m_ArmSwingGainActive = m_ArmSwingGain;
            m_BalanceRollGainActive = m_BalanceRollGain;
            m_BalancePitchGainActive = m_BalancePitchGain;
            m_BalanceEnabledActive = m_BalanceEnabled;
        }

    private:
        int m_KickingLeg;
        float m_KickXOffset;
        float m_KickTargetXOffset;
        float m_KickTargetYOffset;
        float m_KickYOffset;
        float m_KickZOffset;
        float m_KickYawOffset;

        float m_ShiftingBodyDuration;
        float m_KickingDuration;
        float m_RestoringDuration;

        float m_ShiftingBodyDurationActive;
        float m_KickingDurationActive;
        float m_RestoringDurationActive;

        float m_BodyXOffset;
        float m_BodyYOffset;
        float m_BodyZOffset;
        float m_BodyPitchOffset;
        float m_BodyRollOffset;

        float m_BodyXOffsetActive;
        float m_BodyYOffsetActive;
        float m_BodyZOffsetActive;
        float m_BodyPitchOffsetActive;
        float m_BodyRollOffsetActive;

        float m_ArmSwingGain;
        float m_BalanceRollGain;
        float m_BalancePitchGain;
        bool m_BalanceEnabled;

        float m_ArmSwingGainActive;
        float m_BalanceRollGainActive;
        float m_BalancePitchGainActive;
        bool m_BalanceEnabledActive;

        float m_Time;
        int m_Phase;
        bool m_KickingDone;
    };
}


