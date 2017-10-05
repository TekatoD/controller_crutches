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
            PHASE_DONE = 0, // Kick are done
            PHASE_SHIFTING_BODY = 1, // Shift center mass
            PHASE_KICKING = 2, // Kicking
            PHASE_RESTORING = 3  // Go back
        };

        enum {
            RIGHT_LEG = 0,
            LEFT_FEG = 1
        };

    public:
        static Kicking* GetInstance();

        void Kick(int leg, float leg_x_offset, float leg_y_offset,
                  float leg_z_offset, float leg_yaw_offset,
                  float leg_target_x_offset, float leg_target_y_offset);

        bool IsDone() const noexcept;

        void Initialize() override;

        void Process() override;

        void Break();

    private:
        Kicking() = default;

        void UpdateTimeParameters();

        void UpdateActiveParams();

    private:
        int m_kicking_leg{RIGHT_LEG};
        float m_kick_target_x_offset{0.0f};
        float m_kick_target_y_offset{0.0f};
        float m_kick_x_offset{0.0f};
        float m_kick_y_offset{0.0f};
        float m_kick_z_offset{0.0f};
        float m_kick_yaw_offset{0.0f};

        float m_shifting_body_duration{100.0f};
        float m_kicking_duration{30.0f};
        float m_restoring_duration{80.0f};

        float m_cur_shifting_body_duration{0.0f};
        float m_cur_kicking_duration{0.0f};
        float m_cur_restoring_duration{0.0f};

        float m_body_init_x_offset{-10.0f};
        float m_body_init_y_offset{0.0f};
        float m_body_init_z_offset{-5.0f};
        float m_body_init_pitch_offset{radians(-15.0f)};

        float m_cur_body_init_x_offset{0.0f};
        float m_cur_body_init_y_offset{-15.0f};
        float m_cur_body_init_z_offset{0.0f};
        float m_cur_body_init_pitch_offset{0.0f};

        float m_body_x_offset{-20.0f};
        float m_body_y_offset{-15.0f};
        float m_body_z_offset{-5.0f};

        float m_cur_body_x_offset{0.0f};
        float m_cur_body_y_offset{0.0f};
        float m_cur_body_z_offset{0.0f};
        float m_cur_body_pitch_offset{0.0f};

        float m_arm_swing_gain{0.0f};
        float m_balance_roll_gain{0.0f};
        float m_balance_pitch_gain{0.0f};
        bool m_balance_enabled{true};

        float m_cur_arm_swing_gain{0.0f};
        float m_cur_balance_roll_gain{0.0f};
        float m_cur_balance_pitch_gain{0.0f};
        bool m_cur_balance_enabled{true};

        float m_time{0.0f};
        int m_phase{PHASE_DONE};
        bool m_kicking_done{true};
    };
}


