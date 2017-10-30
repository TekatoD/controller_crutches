/*
 *   Walking.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _WALKING_ENGINE_H_
#define _WALKING_ENGINE_H_

#include <string.h>
#include <memory>
#include <fstream>

#include "motion/MotionModule.h"
#include "OdometryCollector.h"

namespace Robot {
    class Walking
            : public MotionModule {
    public:
        enum {
            PHASE0 = 0,
            PHASE1 = 1,
            PHASE2 = 2,
            PHASE3 = 3
        };

    private:
        bool m_debug{false};

        float m_cur_period_time;
        float m_cur_dsp_ratio;
        float m_cur_ssp_ratio;
        float m_cur_x_swap_period_time;
        float m_cur_x_move_period_time;
        float m_cur_y_swap_period_time;
        float m_cur_y_move_period_time;
        float m_cur_z_swap_period_time;
        float m_cur_z_move_period_time;
        float m_cur_a_move_period_time;
        float m_cur_ssp_time;
        float m_cur_ssp_time_start_l;
        float m_cur_ssp_time_end_l;
        float m_cur_ssp_time_start_r;
        float m_cur_ssp_time_end_r;
        float m_cur_phase_time1;
        float m_cur_phase_time2;
        float m_cur_phase_time3;

        float m_cur_x_offset;
        float m_cur_y_offset;
        float m_cur_z_offset;
        float m_cur_r_offset;
        float m_cur_p_offset;
        float m_cur_a_offset;

        float m_cur_x_swap_phase_shift;
        float m_cur_x_swap_amplitude;
        float m_cur_x_swap_amplitude_shift;
        float m_cur_x_move_phase_shift;
        float m_cur_x_move_amplitude;
        float m_cur_x_move_amplitude_shift;
        float m_cur_y_swap_phase_shift;
        float m_cur_y_swap_amplitude;
        float m_cur_y_swap_amplitude_shift;
        float m_cur_y_move_phase_shift;
        float m_cur_y_move_amplitude;
        float m_cur_y_move_amplitude_shift;
        float m_cur_z_swap_phase_shift;
        float m_cur_z_swap_amplitude;
        float m_cur_z_swap_amplitude_shift;
        float m_cur_z_move_phase_shift;
        float m_cur_z_move_amplitude;
        float m_cur_z_move_amplitude_shift;
        float m_cur_a_move_phase_shift;
        float m_cur_a_move_amplitude;
        float m_cur_a_move_amplitude_shift;

        float m_cur_pelvis_offset;
        float m_cur_pelvis_swing;
        float m_cur_hip_pitch_offset;
        float m_cur_arm_swing_gain;

        bool m_ctrl_running;
        bool m_real_running;
        float m_time;

        int m_phase;
        float m_cur_body_swing_y;
        float m_cur_body_swing_z;

        float m_odo_x;
        float m_odo_y;
        float m_odo_theta;

        float m_left_odo_x;
        float m_left_odo_y;
        float m_left_odo_theta;


        float m_right_odo_x;
        float m_right_odo_y;
        float m_right_odo_theta;

        bool m_left_end;
        bool m_right_end;
        bool m_left_start;
        bool m_right_start;

        float m_odo_x_factor;
        float m_odo_y_factor;
        float m_odo_a_factor;

        OdometryCollector m_odometry_collector;

        Walking();

        void UpdateParamTime();

        void UpdateParamMove();

        void UpdateParamBalance();

        Pose2D GetOdoOffset();

        // Walking initial pose
        float m_x_offset;
        float m_y_offset;
        float m_z_offset;
        float m_a_offset;
        float m_p_offset;
        float m_r_offset;

        // Walking control
        float m_period_time;
        float m_dsp_ratio;
        float m_step_fb_ratio;
        float m_x_move_amplitude;
        float m_y_move_amplitude;
        float m_z_move_amplitude;
        float m_a_move_amplitude;
        bool m_move_aim_on;

        // Balance control
        bool m_balance_enable;
        float m_balance_knee_gain;
        float m_balance_ankle_pitch_gain;
        float m_balance_hip_roll_gain;
        float m_balance_ankle_roll_gain;
        float m_y_swap_amplitude;
        float m_z_swap_amplitude;
        float m_arm_swing_gain;
        float m_pelvis_offset;
        float m_hip_pitch_offset;

        int m_p_gain;
        int m_i_gain;
        int m_d_gain;

    public:
        static Walking* GetInstance();

        void Initialize() override;

        void Start();

        void Stop();

        void Process() override;

        bool IsRunning();

        Pose2D GetOdo();

        void ResetOdo(const Pose2D& pose);

        void SetOdo(const Pose2D& pose);

        float GetXOffset() const;

        void SetXOffset(float x_offset);

        float GetYOffset() const;

        void SetYOffset(float y_offset);

        float GetZOffset() const;

        void SetZOffset(float z_offset);

        float GetYawOffset() const;

        void SetYawOffset(float a_offset);

        float GetPitchOffset() const;

        void SetPitchOffset(float p_offset);

        float GetRollOffset() const;

        void SetRollOffset(float r_offset);

        float GetPeriodTime() const;

        void SetPeriodTime(float period_time);

        float GetDSPRatio() const;

        void SetDSPRatio(float dsp_ratio);

        float GetStepFBRatio() const;

        void SetStepFBRatio(float step_fb_ratio);

        float GetXMoveAmplitude() const;

        void SetXMoveAmplitude(float x_move_amplitude);

        float GetYMoveAmplitude() const;

        void SetYMoveAmplitude(float y_move_amplitude);

        float GetZMoveAmplitude() const;

        void SetZMoveAmplitude(float z_move_amplitude);

        float GetAMoveAmplitude() const;

        void SetAMoveAmplitude(float a_move_amplitude);

        bool GetAMoveAimOn() const;

        void SetMoveAimOn(bool move_aim_on);

        bool GetBalanceEnable() const;

        void SetBalanceEnable(bool balance_enable);

        float GetBalanceKneeGain() const;

        void SetBalanceKneeGain(float balance_knee_gain);

        float GetBalanceAnklePitchGain() const;

        void SetBalanceAnklePitchGain(float balance_ankle_pitch_gain);

        float GetBalanceHipRollGain() const;

        void SetBalanceHipRollGain(float balance_hip_roll_gain);

        float GetBalanceAnkleRollGain() const;

        void SetBalanceAnkleRollGain(float balance_ankle_roll_gain);

        float GetYSwapAmplitude() const;

        void SetYSwapAmplitude(float y_swap_amplitude);

        float GetZSwapAmplitude() const;

        void SetZSwapAmplitude(float z_swap_amplitude);

        float GetArmSwingGain() const;

        void SetArmSwingGain(float arm_swing_gain);

        float GetPelvisOffset() const;

        void SetPelvisOffset(float pelvis_offset);

        float GetHipPitchOffset() const;

        void SetHipPitchOffset(float hip_pitch_offset);

        int GetPGain() const;

        void SetPGain(int p_gain);

        int GetIGain() const;

        void SetIGain(int i_gain);

        int GetDGain() const;

        void SetDGain(int d_gain);

        float GetOdoXFactor() const;

        void SetOdoXFactor(float odo_x_factor);

        float GetOdoYFactor() const;

        void SetOdoYFactor(float odo_y_factor);

        float GetOdoAFactor() const;

        void SetOdoAFactor(float odo_a_factor);

        bool IsDebugEnabled() const;

        void EnableDebug(bool debug);
    };
}

#endif
