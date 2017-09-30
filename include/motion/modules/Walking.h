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

#include "minIni.h"
#include "MotionModule.h"
#include "OdometryCollector.h"

#define WALKING_SECTION "Walking Config"
#define INVALID_VALUE   -1024.0

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

        float m_PeriodTime;
        float m_DSP_Ratio;
        float m_SSP_Ratio;
        float m_X_Swap_PeriodTime;
        float m_X_Move_PeriodTime;
        float m_Y_Swap_PeriodTime;
        float m_Y_Move_PeriodTime;
        float m_Z_Swap_PeriodTime;
        float m_Z_Move_PeriodTime;
        float m_A_Move_PeriodTime;
        float m_SSP_Time;
        float m_SSP_Time_Start_L;
        float m_SSP_Time_End_L;
        float m_SSP_Time_Start_R;
        float m_SSP_Time_End_R;
        float m_Phase_Time1;
        float m_Phase_Time2;
        float m_Phase_Time3;

        float m_X_Offset;
        float m_Y_Offset;
        float m_Z_Offset;
        float m_R_Offset;
        float m_P_Offset;
        float m_A_Offset;

        float m_X_Swap_Phase_Shift;
        float m_X_Swap_Amplitude;
        float m_X_Swap_Amplitude_Shift;
        float m_X_Move_Phase_Shift;
        float m_X_Move_Amplitude;
        float m_X_Move_Amplitude_Shift;
        float m_Y_Swap_Phase_Shift;
        float m_Y_Swap_Amplitude;
        float m_Y_Swap_Amplitude_Shift;
        float m_Y_Move_Phase_Shift;
        float m_Y_Move_Amplitude;
        float m_Y_Move_Amplitude_Shift;
        float m_Z_Swap_Phase_Shift;
        float m_Z_Swap_Amplitude;
        float m_Z_Swap_Amplitude_Shift;
        float m_Z_Move_Phase_Shift;
        float m_Z_Move_Amplitude;
        float m_Z_Move_Amplitude_Shift;
        float m_A_Move_Phase_Shift;
        float m_A_Move_Amplitude;
        float m_A_Move_Amplitude_Shift;

        float m_Pelvis_Offset;
        float m_Pelvis_Swing;
        float m_Hip_Pitch_Offset;
        float m_Arm_Swing_Gain;

        bool m_Ctrl_Running;
        bool m_Real_Running;
        float m_Time;

        int m_Phase;
        float m_Body_Swing_Y;
        float m_Body_Swing_Z;

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

        void update_param_time();

        void update_param_move();

        void update_param_balance();

        Pose2D GetOdoOffset();


    public:
        // Walking initial pose
        float X_OFFSET;
        float Y_OFFSET;
        float Z_OFFSET;
        float A_OFFSET;
        float P_OFFSET;
        float R_OFFSET;

        // Walking control
        float PERIOD_TIME;
        float DSP_RATIO;
        float STEP_FB_RATIO;
        float X_MOVE_AMPLITUDE;
        float Y_MOVE_AMPLITUDE;
        float Z_MOVE_AMPLITUDE;
        float A_MOVE_AMPLITUDE;
        bool A_MOVE_AIM_ON;

        bool GO_TO_MODE_ON;
        float GO_TO_X;
        float GO_TO_Y;
        float GO_TO_A;

        // Balance control
        bool BALANCE_ENABLE;
        float BALANCE_KNEE_GAIN;
        float BALANCE_ANKLE_PITCH_GAIN;
        float BALANCE_HIP_ROLL_GAIN;
        float BALANCE_ANKLE_ROLL_GAIN;
        float Y_SWAP_AMPLITUDE;
        float Z_SWAP_AMPLITUDE;
        float ARM_SWING_GAIN;
        float PELVIS_OFFSET;
        float HIP_PITCH_OFFSET;

        int P_GAIN;
        int I_GAIN;
        int D_GAIN;


        float GetXMoveAmplitude() const;

        float GetYMoveAmplitude() const;

        float GetAMoveAmplitude() const;

        bool IsAimMode() const;

        void SetMoveAmplitude(float x, float y, float a, bool aim_mode = false);

        int GetCurrentPhase();

        float GetBodySwingY();

        float GetBodySwingZ();

        virtual ~Walking();

        static Walking* GetInstance() {
            static Walking walking;
            return &walking;
        }

        void Initialize();

        void Start();

        void Stop();

        void Process();

        bool IsRunning();

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

        Pose2D GetOdo();

        void ResetOdo(Pose2D pose);

        void SetOdo(Pose2D pose);
    };
}

#endif
