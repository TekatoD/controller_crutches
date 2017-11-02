/*
 *   BallFollower.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include "math/point_t.h"
#include "ball_tracker_t.h"

#define FOLLOWER_SECTION ("Ball Follower")


namespace drwn {
    enum kicking_action {
        NO_KICKING,
        LEFT_LEG_KICK,
        RIGHT_LEG_KICK
    };

    class ball_follower_t {
    private:
        int m_no_ball_max_count;
        int m_no_ball_count;
        int m_kick_ball_max_count;
        int m_kick_ball_count;

        float m_max_fb_step;
        float m_max_rl_step;
        float m_max_dir_angle;

        float m_kick_top_angle;
        float m_kick_right_angle;
        float m_kick_left_angle;

        float m_follow_max_fb_step;
        float m_follow_min_fb_step;
        float m_follow_max_rl_turn;
        float m_fit_fb_step;
        float m_fit_max_rl_turn;
        float m_unit_fb_step;
        float m_unit_rl_step;
        float m_unit_rl_turn;

        float m_goal_fb_step;
        float m_goal_rl_step;
        float m_goal_rl_turn;
        float m_fb_step;
        float m_rl_step;
        float m_rl_turn;

        float m_tilt_offset;

        float m_aim_tilt_offset;
        float m_aim_rl_step;
        float m_aim_rl_turn;

        kicking_action m_kick_ball;

    public:

        ball_follower_t();

        ~ball_follower_t();

        void process(point_2D_t ball_pos, float angle_top, float angle_bot);

        bool is_no_ball() const;

        kicking_action get_kicking_leg() const;

    };
}
