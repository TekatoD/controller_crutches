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


ball_follower_t* ball_follower_t::get_instance() {
    static ball_follower_t instance;
    return &instance;
}

void ball_follower_t::process(point2d_t ball_pos) {
    using namespace boost::math;

    auto field = field_map_t::get_instance();
    auto walking = walking_t::get_instance();
    auto odo = walking->get_odo();

    const bool look_at_enemy_gate = odo.get_theta() > 0 && odo.get_theta() < constants::pi<float>();

    const float pan = motion_status_t::current_joints.get_angle(joint_data_t::ID_HEAD_PAN);

    const float gate_y_offset = field->get_gate_height() / 2.0f;
    const float gate_x_offset = field->get_field_width() / 2.0f;

    auto calc_angle_to_gate = [gate_x_offset, gate_y_offset, pan, &odo](float x_dir, float y_dir) {
        auto y_diff = y_dir * gate_y_offset - odo.get_y();
        auto x_diff = x_dir * gate_x_offset - odo.get_x();
        return degrees(std::atan2(y_diff, x_diff) - odo.get_theta()) - pan;
    };

    // TODO Check it
    float angle_to_enemy_gate_top = calc_angle_to_gate(1, 1);
    float angle_to_enemy_gate_bot = calc_angle_to_gate(1, -1);
    float angle_to_our_gate_top = calc_angle_to_gate(-1, 1);
    float angle_to_our_gate_bot = calc_angle_to_gate(-1, -1);

    auto normalize = [](float& theta) {
        while (theta < -180.0f) theta += 2.0f * 180.0f;
        while (theta > 180.0f) theta -= 2.0f * 180.0f;
    };

    normalize(angle_to_enemy_gate_top);
    normalize(angle_to_enemy_gate_bot);
    normalize(angle_to_our_gate_top);
    normalize(angle_to_our_gate_bot);

    // TODO Angle to our gate

    if (m_debug) {
        LOG_DEBUG << "BALL FOLLOWER: Processing has been started";
        LOG_DEBUG << "BALL FOLLOWER: ball_pos = (" << ball_pos.X << ", " << ball_pos.Y << ')';
        LOG_DEBUG << "BALL FOLLOWER: angle_to_enemy_gate_top = " << angle_to_enemy_gate_bot;
        LOG_DEBUG << "BALL FOLLOWER: angle_to_enemy_gate_bot = " << angle_to_enemy_gate_top;
        LOG_DEBUG << "BALL FOLLOWER: angle_to_our_gate_top = " << angle_to_our_gate_bot;
        LOG_DEBUG << "BALL FOLLOWER: angle_to_our_gate_bot = " << angle_to_our_gate_top;
    }

    float x_amplitude = walking->get_x_move_amplitude();
    float y_amplitude = walking->get_y_move_amplitude();
    float a_amplitude = walking->get_a_move_amplitude();

    float target_x_amplitude = 0.0f;
    float target_y_amplitude = 0.0f;
    float target_a_amplitude = 0.0f;

    if (ball_pos.X == -1.0 || ball_pos.Y == -1.0) {
        if (m_no_ball_rate.is_passed()) {
            // can not find a ball
            target_x_amplitude = 0.0f;
            target_y_amplitude = 0.0f;
            target_a_amplitude = 0.0f;
//            head_t::get_instance()->move_to_home();
        } else { // Save speed
            target_x_amplitude = x_amplitude;
            target_y_amplitude = 0.0f;
            target_a_amplitude = a_amplitude;
        }
    } else { // Ball found
        m_no_ball_rate.update();

        float pan_range = head_t::get_instance()->get_left_limit_angle();
        float pan_percent = pan / pan_range;
        float tilt = motion_status_t::current_joints.get_angle(joint_data_t::ID_HEAD_TILT);
        float tilt_min = head_t::get_instance()->get_bottom_limit_angle();
        float tilt_range = head_t::get_instance()->get_top_limit_angle() - tilt_min;
        float tilt_percent = std::fabs((tilt - tilt_min) / tilt_range);

        float kicking_angle = std::max(m_slanting_kick_angle, m_straight_kick_angle);

        // If pan between kicking angles
        if (std::fabs(pan) < kicking_angle) {
            if (tilt <= (tilt_min + m_fit_tilt_offset)) {
                if (tilt < (tilt_min + m_kick_tilt_offset)) { // Can kick!
                    target_x_amplitude = 0;
                    target_a_amplitude = 0;

//                    if (m_kick_ball_rate.is_passed()) {  // Ho-ho!
//                        if (pan > 0) {
//                            m_KickBall = LEFT_LEG_KICK; // Left
//                        } else {
//                            m_KickBall = RIGHT_LEG_KICK; // Right
//                        }
//                    } else { // There are no gold! O_o
//                        m_KickBall = NO_KICKING;
//                    }
                } else { // Fit
                    m_kick_ball_rate.update();
                    target_x_amplitude = m_fit_x_amplitude;
                    target_a_amplitude = m_fit_a_amplitude * pan_percent;
                }
            } else {
                m_kick_ball_rate.update();
                target_x_amplitude = std::max(m_follow_max_x_amplitude * tilt_percent, // * (1.0f - std::fabs(pan_percent)),
                                              m_follow_min_x_amplitude);
                target_a_amplitude = m_follow_max_a_amplitude * pan_percent;
            }
        } else { // Out of kicking angles
            m_kick_ball_rate.update();
            target_x_amplitude = 0.0f;
            target_a_amplitude = m_follow_max_a_amplitude * pan_percent;
        }
    }

    if (target_x_amplitude == 0.0 && target_y_amplitude == 0.0 && target_a_amplitude == 0.0) {
        walking->set_x_move_amplitude(0.0);
        walking->set_y_move_amplitude(0.0);
        walking->set_a_move_amplitude(0.0);
        walking->stop();
    } else {
        float x_diff = target_x_amplitude - x_amplitude;
        float y_diff = target_y_amplitude - y_amplitude;
        float a_diff = target_a_amplitude - a_amplitude;
        float x_step = std::copysign(std::min(m_x_accel_step, std::abs(x_diff)), x_diff);
        float y_step = std::copysign(std::min(m_y_accel_step, std::abs(y_diff)), x_diff);
        float a_step = std::copysign(std::min(m_a_accel_step, std::abs(a_diff)), x_diff);
        x_amplitude += x_step;
        y_amplitude += y_step;
        a_amplitude += a_step;
        if (m_debug) {
            LOG_DEBUG << "BALL FOLLOWER: target_x_amplitude = " << target_x_amplitude;
            LOG_DEBUG << "BALL FOLLOWER: target_y_amplitude = " << target_y_amplitude;
            LOG_DEBUG << "BALL FOLLOWER: target_a_amplitude = " << target_a_amplitude;
            LOG_DEBUG << "BALL FOLLOWER: x_step = " << x_step;
            LOG_DEBUG << "BALL FOLLOWER: y_step = " << y_step;
            LOG_DEBUG << "BALL FOLLOWER: a_step = " << a_step;
        }
        walking->set_x_move_amplitude(x_amplitude);
        walking->set_y_move_amplitude(y_amplitude);
        walking->set_a_move_amplitude(a_amplitude);
        walking->start();
    }
}

bool ball_follower_t::is_no_ball() const {
    return m_no_ball_rate.is_passed();
}

bool ball_follower_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void ball_follower_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

steady_rate_t::duration ball_follower_t::get_no_ball_rate() const {
    return m_no_ball_rate.get_duration();
}

void ball_follower_t::set_no_ball_rate(steady_rate_t::duration no_ball_rate) {
    using namespace std::chrono;
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: no_ball_rate = " << duration_cast<milliseconds>(no_ball_rate).count() << "ms";
    m_no_ball_rate.set_duration(no_ball_rate);
}

steady_rate_t::duration ball_follower_t::get_kick_ball_rate() const {
    return m_kick_ball_rate.get_duration();
}

void ball_follower_t::set_kick_ball_rate(steady_rate_t::duration kick_ball_rate) {
    using namespace std::chrono;
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: kick_ball_rate = " << duration_cast<milliseconds>(kick_ball_rate).count() << "ms";
    m_kick_ball_rate.set_duration(kick_ball_rate);
}

float ball_follower_t::get_slanting_kick_angle() const {
    return m_slanting_kick_angle;
}

void ball_follower_t::set_slanting_kick_angle(float slanting_kick_angle) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: slanting_kick_angle = " << slanting_kick_angle;
    m_slanting_kick_angle = slanting_kick_angle;
}

float ball_follower_t::get_straight_kick_angle() const {
    return m_straight_kick_angle;
}

void ball_follower_t::set_straight_kick_angle(float straight_kick_angle) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: straight_kick_angle = " << straight_kick_angle;
    m_straight_kick_angle = straight_kick_angle;
}

float ball_follower_t::get_follow_max_x_amplitude() const {
    return m_follow_max_x_amplitude;
}

void ball_follower_t::set_follow_max_x_amplitude(float follow_max_x_amplitude) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: follow_max_x_amplitude = " << follow_max_x_amplitude;
    m_follow_max_x_amplitude = follow_max_x_amplitude;
}

float ball_follower_t::get_follow_min_x_amplitude() const {
    return m_follow_min_x_amplitude;
}

void ball_follower_t::set_follow_min_x_amplitude(float follow_min_x_amplitude) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: follow_min_x_amplitude = " << follow_min_x_amplitude;
    m_follow_min_x_amplitude = follow_min_x_amplitude;
}

float ball_follower_t::get_follow_max_a_amplitude() const {
    return m_follow_max_a_amplitude;
}

void ball_follower_t::set_follow_max_a_amplitude(float follow_max_a_amplitude) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: follow_max_a_amplitude = " << follow_max_a_amplitude;
    m_follow_max_a_amplitude = follow_max_a_amplitude;
}

float ball_follower_t::get_fit_x_amplitude() const {
    return m_fit_x_amplitude;
}

void ball_follower_t::set_fit_x_amplitude(float fit_x_amplitude) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: fit_x_amplitude = " << fit_x_amplitude;
    m_fit_x_amplitude = fit_x_amplitude;
}

float ball_follower_t::get_fit_a_amplitude() const {
    return m_fit_a_amplitude;
}

void ball_follower_t::set_fit_a_amplitude(float fit_a_amplitude) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: fit_a_amplitude = " << fit_a_amplitude;
    m_fit_a_amplitude = fit_a_amplitude;
}

float ball_follower_t::get_x_accel_step() const {
    return m_x_accel_step;
}

void ball_follower_t::set_x_accel_step(float x_accel_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: x_accel_step = " << x_accel_step;
    m_x_accel_step = x_accel_step;
}

float ball_follower_t::get_y_accel_step() const {
    return m_y_accel_step;
}

void ball_follower_t::set_y_accel_step(float y_accel_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: y_accel_step = " << y_accel_step;
    m_y_accel_step = y_accel_step;
}

float ball_follower_t::get_a_accel_step() const {
    return m_a_accel_step;
}

void ball_follower_t::set_a_accel_step(float a_accel_step) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: a_accel_step = " << a_accel_step;
    m_a_accel_step = a_accel_step;
}

float ball_follower_t::get_kick_tilt_offset() const {
    return m_kick_tilt_offset;
}

void ball_follower_t::set_kick_tilt_offset(float kick_tilt_offset) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: kick_tilt_offset = " << kick_tilt_offset;
    m_kick_tilt_offset = kick_tilt_offset;
}

float ball_follower_t::get_fit_tilt_offset() const {
    return m_fit_tilt_offset;
}

void ball_follower_t::set_fit_tilt_offset(float fit_tilt_offset) {
    if (m_debug) LOG_DEBUG << "BALL FOLLOWER: fit_tilt_offset = " << fit_tilt_offset;
    m_fit_tilt_offset = fit_tilt_offset;
}
