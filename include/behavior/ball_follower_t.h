/*
 *   BallFollower.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include <tool/rate_t.h>
#include "hw/MX28_t.h"
#include "math/point_t.h"

namespace drwn {
    enum class kicking_action_t {
        NO_KICKING,
        LEFT_LEG_KICK,
        RIGHT_LEG_KICK
    };

    class ball_follower_t {
    public:
        static ball_follower_t* get_instance();

        void process(point2d_t ball_pos);

        bool is_no_ball() const;

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

        steady_rate_t::duration get_no_ball_rate() const;

        void set_no_ball_rate(steady_rate_t::duration no_ball_rate);

        steady_rate_t::duration get_kick_ball_rate() const;

        void set_kick_ball_rate(steady_rate_t::duration kick_ball_rate);

        float get_straight_kick_angle() const;

        void set_straight_kick_angle(float straight_kick_angle);

        float get_follow_max_x_amplitude() const;

        void set_follow_max_x_amplitude(float follow_max_x_amplitude);

        float get_follow_min_x_amplitude() const;

        void set_follow_min_x_amplitude(float follow_min_x_amplitude);

        float get_follow_max_a_amplitude() const;

        void set_follow_max_a_amplitude(float follow_max_a_amplitude);

        float get_fit_x_amplitude() const;

        void set_fit_x_amplitude(float fit_x_amplitude);

        float get_fit_a_amplitude() const;

        void set_fit_a_amplitude(float fit_a_amplitude);

        float get_x_accel_step() const;

        void set_x_accel_step(float x_accel_step);

        float get_y_accel_step() const;

        void set_y_accel_step(float y_accel_step);

        float get_a_accel_step() const;

        void set_a_accel_step(float a_accel_step);

        float get_kick_tilt_offset() const;

        void set_kick_tilt_offset(float kick_tilt_offset);

        float get_fit_tilt_offset() const;

        void set_fit_tilt_offset(float fit_tilt_offset);

        float get_aim_y_amplitude() const;

        void set_aim_y_amplitude(float aim_y_amplitude);

        float get_aim_a_amplitude() const;

        void set_aim_a_amplitude(float aim_a_amplitude);

        float get_aim_tilt_offset() const;

        void set_aim_tilt_offset(float aim_kick_offset);

    private:
        ball_follower_t() = default;

        void kick_ball();

        void calculate_angles_to_gait();

    public:
        float get_aim_x_amplitude() const;

        void set_aim_x_amplitude(float aim_x_amplitude);

    private:
        bool m_debug{false};

        steady_rate_t m_no_ball_rate{std::chrono::seconds(2)};
        steady_rate_t m_kick_ball_rate{std::chrono::milliseconds(500)};

        float m_straight_kick_angle{90.0f};

        float m_follow_max_x_amplitude{7.0f};
        float m_follow_min_x_amplitude{5.0f};
        float m_follow_max_a_amplitude{5.0f};
        float m_fit_x_amplitude{3.0f};
        float m_fit_a_amplitude{3.0f};
        float m_aim_x_amplitude{0.0f};
        float m_aim_y_amplitude{7.0f};
        float m_aim_a_amplitude{7.0f};

        float m_x_accel_step{0.3f};
        float m_y_accel_step{0.3f};
        float m_a_accel_step{1.0f};

        float m_kick_tilt_offset{3.0f};//{MX28_t::RATIO_VALUE2DEGREES};
        float m_fit_tilt_offset{5.0f};
        float m_aim_tilt_offset{6.0f};

        point2d_t m_local_ball;

        float m_angle_to_enemy_gate_top{0.0f};
        float m_angle_to_enemy_gate_bot{0.0f};
        float m_angle_to_our_gate_top{0.0f};
        float m_angle_to_our_gate_bot{0.0f};
        float m_angle_to_enemy_gate_center{0.0f};
//        float m_angle_to_enemy__center{0.0f};
    };
}
