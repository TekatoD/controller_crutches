/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <iostream>
#include "hw/MX28_t.h"
#include "motion/modules/head_t.h"
#include "motion/modules/walking_t.h"
#include "ball_follower_t.h"
#include "motion/motion_status_t.h"


using namespace drwn;


ball_follower_t::ball_follower_t() {
    m_no_ball_max_count = 10;
    m_no_ball_count = m_no_ball_max_count;
    m_kick_ball_max_count = 10;
    m_kick_ball_count = 0;

    m_kick_top_angle = -5.0;
    m_kick_right_angle = -30.0;
    m_kick_left_angle = 30.0;

    m_follow_max_fb_step = 30.0;
    m_follow_min_fb_step = 5.0;
    m_follow_max_rl_turn = 35.0;
    m_fit_fb_step = 3.0;
    m_fit_max_rl_turn = 35.0;
    m_unit_fb_step = 0.3;
    m_unit_rl_step = 0.3;
    m_unit_rl_turn = 1.0;

    m_goal_fb_step = 0;
    m_goal_rl_step = 0;
    m_goal_rl_turn = 0;
    m_fb_step = 0;
    m_rl_step = 0;
    m_rl_turn = 0;
    m_tilt_offset = MX28_t::RATIO_VALUE2DEGREES;
    m_kick_ball = NO_KICKING;

    m_aim_tilt_offset = 15;
    m_aim_rl_turn = 10;
    m_aim_rl_step = 20;
}


ball_follower_t::~ball_follower_t() {
}


void ball_follower_t::process(point_2D_t ball_pos,
                              float angle_top,
                              float angle_bot) {
    bool aim = false;

    if (ball_pos.X == -1.0 || ball_pos.Y == -1.0) {
        m_kick_ball = NO_KICKING;

        if (m_no_ball_count > m_no_ball_max_count) {
            // can not find a ball
            m_goal_fb_step = 0;
            m_goal_rl_turn = 0;
            head_t::get_instance()->move_to_home();
        } else {
            m_no_ball_count++;
        }
    } else {
        m_no_ball_count = 0;

        float pan = motion_status_t::m_current_joints.set_angle(joint_data_t::ID_HEAD_PAN);
        float pan_range = head_t::get_instance()->get_left_limit_angle();
        float pan_percent = pan / pan_range;

        float tilt = motion_status_t::m_current_joints.set_angle(joint_data_t::ID_HEAD_TILT);
        float tilt_min = head_t::get_instance()->get_bottom_limit_angle();
        float tilt_range = head_t::get_instance()->get_top_limit_angle() - tilt_min;
        float tilt_percent = (tilt - tilt_min) / tilt_range;
        if (tilt_percent < 0)
            tilt_percent = -tilt_percent;
        if (pan > m_kick_right_angle && pan < m_kick_left_angle) {
            if (tilt <= (tilt_min + m_aim_tilt_offset) && (angle_top > 0 || angle_bot < 0)) {
                m_kick_ball_count = 0;
                aim = true;
                m_goal_fb_step = 0;
                if (angle_top > 0) {
                    m_goal_rl_step = -m_aim_rl_step;
                    m_goal_rl_turn = m_aim_rl_turn * pan_percent;
                } else {
                    m_goal_rl_step = m_aim_rl_step;
                    m_goal_rl_turn = -m_aim_rl_turn * pan_percent;
                }
            } else if (tilt <= (tilt_min + m_tilt_offset)) {
                if (ball_pos.Y < m_kick_top_angle) {
                    m_goal_fb_step = 0;
                    m_goal_rl_step = 0;
                    m_goal_rl_turn = 0;

                    if (m_kick_ball_count >= m_kick_ball_max_count) {
                        m_fb_step = 0;
                        m_rl_step = 0;
                        m_rl_turn = 0;
                        if (pan > 0) {
                            m_kick_ball = LEFT_LEG_KICK; // Left
                        } else {
                            m_kick_ball = RIGHT_LEG_KICK; // Right
                        }
                    } else {
                        m_kick_ball = NO_KICKING;
                    }
                } else {
                    m_kick_ball_count = 0;
                    m_kick_ball = NO_KICKING;
                    m_goal_fb_step = m_fit_fb_step;
                    m_goal_rl_step = 0;
                    m_goal_rl_turn = m_fit_max_rl_turn * pan_percent;
                }
            } else {
                m_kick_ball_count = 0;
                m_kick_ball = NO_KICKING;
                m_goal_fb_step = m_follow_max_fb_step * tilt_percent;
                if (m_goal_fb_step < m_follow_min_fb_step)
                    m_goal_fb_step = m_follow_min_fb_step;
                m_goal_rl_step = 0;
                m_goal_rl_turn = m_follow_max_rl_turn * pan_percent;
            }
        } else {
            m_kick_ball_count = 0;
            m_kick_ball = NO_KICKING;
            m_goal_fb_step = 0;
            m_goal_rl_step = 0;
            m_goal_rl_turn = m_follow_max_rl_turn * pan_percent;
        }
    }

    if (m_goal_fb_step == 0 && m_goal_rl_step == 0 && m_goal_rl_step == 0 &&
            m_fb_step == 0 && m_rl_turn == 0 && m_rl_step == 0) {
        if (walking_t::get_instance()->is_running()) {
            walking_t::get_instance()->stop();
        } else {
            if (m_kick_ball_count < m_kick_ball_max_count)
                m_kick_ball_count++;
        }
    } else {
        if (!walking_t::get_instance()->is_running()) {

            m_fb_step = 0;
            m_rl_step = 0;
            m_rl_turn = 0;
            m_kick_ball_count = 0;
            m_kick_ball = NO_KICKING;

            walking_t::get_instance()->set_x_move_amplitude(m_fb_step);
            walking_t::get_instance()->set_y_move_amplitude(m_rl_step);
            walking_t::get_instance()->set_a_move_amplitude(m_rl_turn);
            walking_t::get_instance()->start();
        } else {
            if (m_fb_step < m_goal_fb_step)
                m_fb_step += m_unit_fb_step;
            else if (m_fb_step > m_goal_fb_step)
                m_fb_step = m_goal_fb_step;
            walking_t::get_instance()->set_x_move_amplitude(m_fb_step);

            if (m_goal_rl_step > 0) {
                if (m_rl_step < m_goal_rl_step)
                    m_rl_step += m_unit_rl_step;
                else if (m_fb_step > m_goal_rl_step)
                    m_rl_step = m_goal_rl_step;
            } else {
                if (m_rl_step > -m_goal_rl_step)
                    m_rl_step -= m_unit_rl_step;
                else if (m_fb_step < -m_goal_rl_step)
                    m_rl_step = -m_goal_rl_step;
            }
            walking_t::get_instance()->set_y_move_amplitude(m_rl_step);

            if (m_rl_turn < m_goal_rl_turn)
                m_rl_turn += m_unit_rl_turn;
            else if (m_rl_turn > m_goal_rl_turn)
                m_rl_turn -= m_unit_rl_turn;
            walking_t::get_instance()->set_a_move_amplitude(m_rl_turn);
            walking_t::get_instance()->set_move_aim_on(aim);
        }
    }
}

bool ball_follower_t::is_no_ball() const {
    return m_no_ball_count >= m_no_ball_max_count;
}

kicking_action ball_follower_t::get_kicking_leg() const {
    return m_kick_ball;
}
