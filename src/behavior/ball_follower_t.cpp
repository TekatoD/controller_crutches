/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <iostream>
#include <log/trivial_logger_t.h>
#include <math/angle_tools.h>
#include <localization/field_map_t.h>
#include <boost/math/constants/constants.hpp>
#include "hw/MX28_t.h"
#include "motion/modules/head_t.h"
#include "motion/modules/walking_t.h"
#include "behavior/ball_follower_t.h"
#include "motion/motion_status_t.h"


using namespace drwn;


void ball_follower_t::process(point2d_t ball_pos) {
    using namespace boost::math;

    // Calculate angles to gate
    auto field = field_map_t::get_instance();
    auto odo = walking_t::get_instance()->get_odo();

    float outside_gate_space = (field->get_field_height() - field->get_gate_height()) / 2.0f;
    float gate_y_top = field->get_field_height() - outside_gate_space;
    float gate_y_bot = gate_y_top - field->get_gate_height();

    bool look_at_enemy_gate = odo.get_theta() > 0 && odo.get_theta() < constants::pi<float>();

    float pan = motion_status_t::current_joints.get_angle(joint_data_t::ID_HEAD_PAN);
    // TODO Check it
    float angle_to_gate_top = degrees(atan2f(gate_y_top - odo.get_y(), field->get_field_width() - odo.get_x()) -
                              odo.get_theta());
    float angle_to_gate_bot = degrees(atan2f(gate_y_bot - odo.get_y(), field->get_field_width() - odo.get_x()) -
                              odo.get_theta());
    angle_to_gate_top -= pan;
    angle_to_gate_bot -= pan;

    auto normalize = [](float& theta) {
        while (theta < -180.0f) theta += 2.0f * 180.0f;
        while (theta > 180.0f) theta -= 2.0f * 180.0f;
    };

    normalize(angle_to_gate_top);
    normalize(angle_to_gate_bot);

    // TODO Angle to our gate
    
    if (m_debug) {
        LOG_DEBUG << "BALL FOLLOWER: Processing has been started";
        LOG_DEBUG << "BALL FOLLOWER: ball_pos = (" << ball_pos.X << ", " << ball_pos.Y << ')';
        LOG_DEBUG << "BALL FOLLOWER: angle_to_gate_bot = " << angle_to_gate_top;
        LOG_DEBUG << "BALL FOLLOWER: angle_to_gate_top = " << angle_to_gate_bot;
    }

    if (ball_pos.X == -1.0 || ball_pos.Y == -1.0) {
        if (m_no_ball_rate.is_passed()) {
            // can not find a ball
            m_cur_goal_x_step = 0;
            m_cur_goal_z_turn = 0;
            head_t::get_instance()->move_to_home();
        }
    } else {
        m_no_ball_rate.update();

        float pan_range = head_t::get_instance()->get_left_limit_angle();
        float pan_percent = pan / pan_range;

        float tilt = motion_status_t::current_joints.get_angle(joint_data_t::ID_HEAD_TILT);
        float tilt_min = head_t::get_instance()->get_bottom_limit_angle();
        float tilt_range = head_t::get_instance()->get_top_limit_angle() - tilt_min;
        float tilt_percent = fabsf((tilt - tilt_min) / tilt_range);

        // Pan between kicking angles
        if (pan > m_allowable_angle_to_kicking && pan < -m_allowable_angle_to_kicking) {
            if (tilt <= (tilt_min + m_aim_tilt_offset) && (angle_to_gate_top > 0 || angle_to_gate_bot < 0)) {
                m_kick_ball_count = 0;
                aim = true;
                m_cur_goal_x_step = 0;
                if (angle_to_gate_top > 0) {
                    m_cur_goal_y_step = -m_aim_z_step;
                    m_cur_goal_z_turn = m_aim_y_turn * pan_percent;
                } else {
                    m_cur_goal_y_step = m_aim_z_step;
                    m_cur_goal_z_turn = -m_aim_y_turn * pan_percent;
                }
            } else if (tilt <= (tilt_min + m_tilt_offset)) {
                if (ball_pos.Y < m_straight_kick_angle) {
                    m_cur_goal_x_step = 0;
                    m_cur_goal_y_step = 0;
                    m_cur_goal_z_turn = 0;

                    if (m_kick_ball_count >= m_kick_ball_max_count) {
                        m_cur_x_amplitude = 0;
                        m_cur_y_amplitude = 0;
                        m_cur_a_amplitude = 0;
                        if (pan > 0) {
                            m_kick_ball = kicking_action_t::LEFT_LEG_KICK; // Left
                        } else {
                            m_kick_ball = kicking_action_t::RIGHT_LEG_KICK; // Right
                        }
                    } else {
                        m_kick_ball = kicking_action_t::NO_KICKING;
                    }
                } else {
                    m_kick_ball_count = 0;
                    m_kick_ball = kicking_action_t::NO_KICKING;
                    m_cur_goal_x_step = m_fit_x_amplitude;
                    m_cur_goal_y_step = 0;
                    m_cur_goal_z_turn = m_fit_a_amplitude * pan_percent;
                }
            } else {
                m_kick_ball_count = 0;
                m_kick_ball = kicking_action_t::NO_KICKING;
                m_cur_goal_x_step = m_follow_max_x_aplitude * tilt_percent;
                if (m_cur_goal_x_step < m_follow_min_x_amplitude)
                    m_cur_goal_x_step = m_follow_min_x_amplitude;
                m_cur_goal_y_step = 0;
                m_cur_goal_z_turn = m_follow_max_a_amplitude * pan_percent;
            }
        } else {
            m_kick_ball_count = 0;
            m_kick_ball = kicking_action_t::NO_KICKING;
            m_cur_goal_x_step = 0;
            m_cur_goal_y_step = 0;
            m_cur_goal_z_turn = m_follow_max_a_amplitude * pan_percent;
        }
    }

    if (m_cur_goal_x_step == 0 && m_cur_goal_y_step == 0 && m_cur_goal_y_step == 0 &&
            m_cur_x_amplitude == 0 && m_cur_a_amplitude == 0 && m_cur_y_amplitude == 0) {
        if (walking_t::get_instance()->is_running()) {
            walking_t::get_instance()->stop();
        } else {
            if (m_kick_ball_count < m_kick_ball_max_count)
                m_kick_ball_count++;
        }
    } else {
        if (!walking_t::get_instance()->is_running()) {

            m_cur_x_amplitude = 0;
            m_cur_y_amplitude = 0;
            m_cur_a_amplitude = 0;
            m_kick_ball_count = 0;
            m_kick_ball = kicking_action_t::NO_KICKING;

            walking_t::get_instance()->set_x_move_amplitude(m_cur_x_amplitude);
            walking_t::get_instance()->set_y_move_amplitude(m_cur_y_amplitude);
            walking_t::get_instance()->set_a_move_amplitude(m_cur_a_amplitude);
            walking_t::get_instance()->start();
        } else {
            if (m_cur_x_amplitude < m_cur_goal_x_step)
                m_cur_x_amplitude += m_unit_x_step;
            else if (m_cur_x_amplitude > m_cur_goal_x_step)
                m_cur_x_amplitude = m_cur_goal_x_step;
            walking_t::get_instance()->set_x_move_amplitude(m_cur_x_amplitude);

            if (m_cur_goal_y_step > 0) {
                if (m_cur_y_amplitude < m_cur_goal_y_step)
                    m_cur_y_amplitude += m_unit_y_step;
                else if (m_cur_x_amplitude > m_cur_goal_y_step)
                    m_cur_y_amplitude = m_cur_goal_y_step;
            } else {
                if (m_cur_y_amplitude > -m_cur_goal_y_step)
                    m_cur_y_amplitude -= m_unit_y_step;
                else if (m_cur_x_amplitude < -m_cur_goal_y_step)
                    m_cur_y_amplitude = -m_cur_goal_y_step;
            }
            walking_t::get_instance()->set_y_move_amplitude(m_cur_y_amplitude);

            if (m_cur_a_amplitude < m_cur_goal_z_turn)
                m_cur_a_amplitude += m_unit_a_step;
            else if (m_cur_a_amplitude > m_cur_goal_z_turn)
                m_cur_a_amplitude -= m_unit_a_step;
            walking_t::get_instance()->set_a_move_amplitude(m_cur_a_amplitude);
            walking_t::get_instance()->set_move_aim_on(aim);
        }
    }
}

bool ball_follower_t::is_no_ball() const {
    return m_no_ball_count >= m_no_ball_max_count;
}

kicking_action_t ball_follower_t::get_kicking_action() const noexcept {
    return m_kick_ball;
}

ball_follower_t* ball_follower_t::get_instance() {
    static ball_follower_t instance;
    return &instance;
}

int ball_follower_t::get_no_ball_max_count() const noexcept {
    return m_no_ball_max_count;
}

void ball_follower_t::set_no_ball_max_count(int no_ball_max_count) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: no_ball_max_count = " << no_ball_max_count;
    m_no_ball_max_count = no_ball_max_count;
}

int ball_follower_t::get_kick_ball_max_count() const noexcept {
    return m_kick_ball_max_count;
}

void ball_follower_t::set_kick_ball_max_count(int kick_ball_max_count) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: kick_ball_max_count = " << kick_ball_max_count;
    m_kick_ball_max_count = kick_ball_max_count;
}

float ball_follower_t::get_kick_top_angle() const noexcept {
    return m_straight_kick_angle;
}

void ball_follower_t::set_kick_top_angle(float kick_top_angle) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: kick_top_angle = " << kick_top_angle;
    m_straight_kick_angle = kick_top_angle;
}

float ball_follower_t::get_kick_right_angle() const noexcept {
    return m_allowable_angle_to_kicking;
}

void ball_follower_t::set_kick_right_angle(float kick_right_angle) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: kick_right_angle = " << kick_right_angle;
    m_allowable_angle_to_kicking = kick_right_angle;
}

float ball_follower_t::get_kick_left_angle() const noexcept {
    return m_kick_left_angle;
}

void ball_follower_t::set_kick_left_angle(float kick_left_angle) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: kick_left_angle = " << kick_left_angle;
    m_kick_left_angle = kick_left_angle;
}

float ball_follower_t::get_follow_max_x_step() const noexcept {
    return m_follow_max_x_aplitude;
}

void ball_follower_t::set_follow_max_x_step(float follow_max_x_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: follow_max_x_step = " << follow_max_x_step;
    m_follow_max_x_aplitude = follow_max_x_step;
}

float ball_follower_t::get_follow_min_x_step() const noexcept {
    return m_follow_min_x_amplitude;
}

void ball_follower_t::set_follow_min_x_step(float follow_min_x_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: follow_min_x_step = " << follow_min_x_step;
    m_follow_min_x_amplitude = follow_min_x_step;
}

float ball_follower_t::get_follow_max_z_turn() const noexcept {
    return m_follow_max_a_amplitude;
}

void ball_follower_t::set_follow_max_z_turn(float follow_max_z_turn) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: follow_max_z_turn = " << follow_max_z_turn;
    m_follow_max_a_amplitude = follow_max_z_turn;
}

float ball_follower_t::get_fit_x_step() const noexcept {
    return m_fit_x_amplitude;
}

void ball_follower_t::set_fit_x_step(float fit_x_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: follow_max_z_turn = " << fit_x_step;
    m_fit_x_amplitude = fit_x_step;
}

float ball_follower_t::get_fit_max_z_turn() const noexcept {
    return m_fit_a_amplitude;
}

void ball_follower_t::set_fit_max_z_turn(float fit_max_z_turn) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: fit_max_z_turn = " << fit_max_z_turn;
    m_fit_a_amplitude = fit_max_z_turn;
}

float ball_follower_t::get_unit_x_step() const noexcept {
    return m_unit_x_step;
}

void ball_follower_t::set_unit_x_step(float unit_x_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: unit_x_step = " << unit_x_step;
    m_unit_x_step = unit_x_step;
}

float ball_follower_t::get_unit_y_step() const noexcept {
    return m_unit_y_step;
}

void ball_follower_t::set_unit_y_step(float unit_y_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: unit_y_step = " << unit_y_step;
    m_unit_y_step = unit_y_step;
}

float ball_follower_t::get_unit_z_turn() const noexcept {
    return m_unit_a_step;
}

void ball_follower_t::set_unit_z_turn(float unit_z_turn) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: unit_z_turn = " << unit_z_turn;
    m_unit_a_step = unit_z_turn;
}

float ball_follower_t::get_goal_x_step() const noexcept {
    return m_cur_goal_x_step;
}

void ball_follower_t::set_goal_x_step(float goal_x_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: goal_x_step = " << goal_x_step;
    m_cur_goal_x_step = goal_x_step;
}

float ball_follower_t::get_goal_y_step() const noexcept {
    return m_cur_goal_y_step;
}

void ball_follower_t::set_goal_y_step(float goal_y_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: goal_y_step = " << goal_y_step;
    m_cur_goal_y_step = goal_y_step;
}

float ball_follower_t::get_goal_z_turn() const noexcept {
    return m_cur_goal_z_turn;
}

void ball_follower_t::set_goal_z_turn(float goal_z_turn) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: goal_z_turn = " << goal_z_turn;
    m_cur_goal_z_turn = goal_z_turn;
}

float ball_follower_t::get_x_step() const noexcept {
    return m_cur_x_amplitude;
}

void ball_follower_t::set_x_step(float x_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: x_step = " << x_step;
    m_cur_x_amplitude = x_step;
}

float ball_follower_t::get_y_step() const noexcept {
    return m_cur_y_amplitude;
}

void ball_follower_t::set_y_step(float y_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: y_step = " << y_step;
    m_cur_y_amplitude = y_step;
}

float ball_follower_t::get_z_turn() const noexcept {
    return m_cur_a_amplitude;
}

void ball_follower_t::set_z_turn(float z_turn) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: z_turn = " << z_turn;
    m_cur_a_amplitude = z_turn;
}

float ball_follower_t::get_tilt_offset() const noexcept {
    return m_tilt_offset;
}

void ball_follower_t::set_tilt_offset(float tilt_offset) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: tilt_offset = " << tilt_offset;
    m_tilt_offset = tilt_offset;
}

float ball_follower_t::get_aim_tilt_offset() const noexcept {
    return m_aim_tilt_offset;
}

void ball_follower_t::set_aim_tilt_offset(float aim_tilt_offset) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: aim_tilt_offset = " << aim_tilt_offset;
    m_aim_tilt_offset = aim_tilt_offset;
}

float ball_follower_t::get_aim_y_turn() const noexcept {
    return m_aim_y_turn;
}

void ball_follower_t::set_aim_y_turn(float aim_y_turn) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: aim_y_turn = " << aim_y_turn;
    m_aim_y_turn = aim_y_turn;
}

float ball_follower_t::get_aim_z_step() const noexcept {
    return m_aim_z_step;
}

void ball_follower_t::set_aim_z_step(float aim_z_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: aim_z_step = " << aim_z_step;
    m_aim_z_step = aim_z_step;
}

bool ball_follower_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void ball_follower_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}
