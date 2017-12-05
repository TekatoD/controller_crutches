/**
 *  @autor arssivka
 *  @date 9/20/17
 */

#pragma once


#include <utility>
#include <motion/motion_module_t.h>
#include <math/angle_tools.h>
#include <motion/kinematics_t.h>
#include <hw/MX28_t.h>

namespace drwn {
    class kicking_t
            : public motion_module_t {
    public:
        enum {
            PHASE_DONE = 0, // Kick are done
            PHASE_SHIFTING_BODY = 1, // Shift center mass
            PHASE_KICKING = 2, // Kicking
            PHASE_RESTORING = 3  // Go back
        };

        enum {
            RIGHT_LEG = 0,
            LEFT_LEG = 1
        };

    public:
        static kicking_t* get_instance();

        void kick();

        bool is_running() const noexcept;

        void initialize() override;

        void process() override;

        void stop();

        int get_kicking_leg() const noexcept;

        void set_kicking_leg(int kicking_leg) noexcept;

        // Kick endpoint
        float get_kick_target_x_offset() const noexcept;

        void set_kick_target_x_offset(float kick_target_x_offset) noexcept;

        float get_kick_target_y_offset() const noexcept;

        void set_kick_target_y_offset(float kick_target_y_offset) noexcept;

        // Kick starting point
        float get_kick_x_offset() const noexcept;

        void set_kick_x_offset(float kick_x_offset) noexcept;

        float get_kick_y_offset() const noexcept;

        void set_kick_y_offset(float kick_y_offset) noexcept;

        float get_kick_z_offset() const noexcept;

        void set_kick_z_offset(float kick_z_offset) noexcept;

        float get_kick_yaw_offset() const noexcept;

        void set_kick_yaw_offset(float kick_yaw_offset) noexcept;

        float get_shifting_body_duration() const noexcept;

        void set_shifting_body_duration(float shifting_body_duration) noexcept;

        float get_kicking_duration() const noexcept;

        void set_kicking_duration(float kicking_duration) noexcept;

        float get_restoring_duration() const noexcept;

        void set_restoring_duration(float restoring_duration) noexcept;

        // Body position before shifting COM
        float get_body_init_x_offset() const noexcept;

        void set_body_init_x_offset(float body_init_x_offset) noexcept;

        float get_body_init_y_offset() const noexcept;

        void set_body_init_y_offset(float body_init_y_offset) noexcept;

        float get_body_init_z_offset() const noexcept;

        void set_body_init_z_offset(float body_init_z_offset) noexcept;

        float get_body_init_pitch_offset() const noexcept;

        void set_body_init_pitch_offset(float body_init_pitch_offset) noexcept;

        // Body position after shifting COM
        float get_body_x_offset() const noexcept;

        void set_body_x_offset(float body_x_offset) noexcept;

        float get_body_y_offset() const noexcept;

        void set_body_y_offset(float body_y_offset) noexcept;

        float get_body_z_offset() const noexcept;

        void set_body_z_offset(float body_z_offset) noexcept;

        float get_arm_swing_amplitude() const noexcept;

        void set_arm_swing_amplitude(float arm_swing_amplitude) noexcept;

        float get_balance_roll_gain() const noexcept;

        void set_balance_roll_gain(float balance_roll_gain) noexcept;

        float get_balance_pitch_gain() const noexcept;

        void set_balance_pitch_gain(float balance_pitch_gain) noexcept;

        bool get_balance_enabled() const noexcept;

        void set_balance_enabled(bool balance_enabled) noexcept;

        float get_legs_y_offset() const noexcept ;

        void set_legs_y_offset(float legs_y_offset);

        float get_arm_spread_offset() const noexcept ;

        void set_arm_spread_offset(float arm_spread_amplitude);

        float get_elbow_offset() const noexcept;

        void set_elbow_offset(float arm_elbow_offset);

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        kicking_t() = default;

        void update_time_parameters();

        void update_active_params() noexcept;

    private:
        bool m_debug{false};

        int m_kicking_leg{RIGHT_LEG};
        float m_kick_target_x_offset{0.0f};
        float m_kick_target_y_offset{0.0f};
        float m_kick_x_offset{0.0f};
        float m_kick_y_offset{0.0f};
        float m_kick_z_offset{5.0f};
        float m_kick_yaw_offset{0.0f};

        float m_shifting_body_duration{450.0f};
        float m_kicking_duration{300.0f};
        float m_restoring_duration{450.0f};

        float m_body_init_x_offset{-10.0f};
        float m_body_init_y_offset{0.0f}; // Unused
        float m_body_init_z_offset{5.0f};
        float m_body_init_pitch_offset{radians(13.0f)};
        float m_legs_y_offset{5};

        float m_body_x_offset{-15.0f};
        float m_body_y_offset{-25.0f};
        float m_body_z_offset{7.0f};

        float m_arm_swing_amplitude{radians(20.0)};
        float m_arm_spread_offset{radians(-15.0)};
        float m_elbow_offset{radians(-25)};
        float m_balance_roll_gain{0.0f};
        float m_balance_pitch_gain{0.0f};
        bool m_balance_enabled{true};

        int m_cur_kicking_leg{RIGHT_LEG};
        float m_cur_kick_target_x_offset{0.0f};
        float m_cur_kick_target_y_offset{0.0f};
        float m_cur_kick_x_offset{0.0f};
        float m_cur_kick_y_offset{0.0f};
        float m_cur_kick_z_offset{0.0f};
        float m_cur_kick_yaw_offset{0.0f};

        float m_cur_shifting_body_duration{0.0f};
        float m_cur_kicking_duration{0.0f};
        float m_cur_restoring_duration{0.0f};

        float m_cur_body_init_x_offset{0.0f};
        float m_cur_body_init_y_offset{0.0f};
        float m_cur_body_init_z_offset{0.0f};
        float m_cur_body_init_pitch_offset{0.0f};
        float m_cur_legs_y_offset{0.0f};

        float m_cur_body_x_offset{0.0f};
        float m_cur_body_y_offset{0.0f};
        float m_cur_body_z_offset{0.0f};
        float m_cur_body_pitch_offset{0.0f};

        float m_cur_arm_swing_amplitude{0.0f};
        float m_cur_arm_spread_offset{0.0f};
        float m_cur_elbow_swing_offset{0.0f};
        float m_cur_balance_roll_gain{0.0f};
        float m_cur_balance_pitch_gain{0.0f};
        bool m_cur_balance_enabled{true};

        float m_time{0.0f};
        int m_phase{PHASE_DONE};
        bool m_done{true};
    };
}

