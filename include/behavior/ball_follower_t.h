/*
 *   BallFollower.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

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

        void process(point2d_t ball_pos, float angle_top, float angle_bot);

        bool is_no_ball() const;

        kicking_action_t get_kicking_action() const noexcept;

        int get_no_ball_max_count() const noexcept;

        void set_no_ball_max_count(int no_ball_max_count);

        int get_kick_ball_max_count() const noexcept;

        void set_kick_ball_max_count(int kick_ball_max_count);

        float get_kick_top_angle() const noexcept;

        void set_kick_top_angle(float kick_top_angle);

        float get_kick_right_angle() const noexcept;

        void set_kick_right_angle(float kick_right_angle);

        float get_kick_left_angle() const noexcept;

        void set_kick_left_angle(float kick_left_angle);

        float get_follow_max_x_step() const noexcept;

        void set_follow_max_x_step(float follow_max_x_step);

        float get_follow_min_x_step() const noexcept;

        void set_follow_min_x_step(float follow_min_x_step);

        float get_follow_max_z_turn() const noexcept;

        void set_follow_max_z_turn(float follow_max_z_turn);

        float get_fit_x_step() const noexcept;

        void set_fit_x_step(float fit_x_step);

        float get_fit_max_z_turn() const noexcept;

        void set_fit_max_z_turn(float fit_max_z_turn);

        float get_unit_x_step() const noexcept;

        void set_unit_x_step(float unit_x_step);

        float get_unit_y_step() const noexcept;

        void set_unit_y_step(float unit_y_step);

        float get_unit_z_turn() const noexcept;

        void set_unit_z_turn(float unit_z_turn);

        float get_goal_x_step() const noexcept;

        void set_goal_x_step(float goal_x_step);

        float get_goal_y_step() const noexcept;

        void set_goal_y_step(float goal_y_step);

        float get_goal_z_turn() const noexcept;

        void set_goal_z_turn(float goal_z_turn);

        float get_x_step() const noexcept;

        void set_x_step(float x_step);

        float get_y_step() const noexcept;

        void set_y_step(float y_step);

        float get_z_turn() const noexcept;

        void set_z_turn(float z_turn);

        float get_tilt_offset() const noexcept;

        void set_tilt_offset(float tilt_offset);

        float get_aim_tilt_offset() const noexcept;

        void set_aim_tilt_offset(float aim_tilt_offset);

        float get_aim_y_turn() const noexcept;

        void set_aim_y_turn(float aim_y_turn);

        float get_aim_z_step() const noexcept;

        void set_aim_z_step(float aim_z_step);

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        ball_follower_t() = default;

    private:
        bool m_debug{false};

        int m_no_ball_max_count{10};
        int m_no_ball_count{m_no_ball_max_count};
        int m_kick_ball_max_count{0};
        int m_kick_ball_count{0};

        float m_kick_top_angle{-5.0f};
        float m_kick_right_angle{-30.0f};
        float m_kick_left_angle{30.0f};

        float m_follow_max_x_step{30.0f};
        float m_follow_min_x_step{5.0f};
        float m_follow_max_z_turn{35.0f};
        float m_fit_x_step{3.0f};
        float m_fit_max_z_turn{35.0f};
        float m_unit_x_step{0.3f};
        float m_unit_y_step{0.3f};
        float m_unit_z_turn{1.0f};

        float m_goal_x_step{0.0f};
        float m_goal_y_step{0.0f};
        float m_goal_z_turn{0.0f};
        float m_x_step{0.0f};
        float m_y_step{0.0f};
        float m_z_turn{0.0f};
        float m_tilt_offset{MX28_t::RATIO_VALUE2DEGREES};
        kicking_action_t m_kick_ball{kicking_action_t::NO_KICKING};

        float m_aim_tilt_offset{15};
        float m_aim_y_turn{10};
        float m_aim_z_step{20};
    };
}
