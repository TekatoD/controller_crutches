/**
 *  @autor arssivka
 *  @date 5/3/17
 */

#include <math/angle_tools.h>
#include <log/trivial_logger_t.h>
#include "behavior/go_to_t.h"
#include "motion/modules/walking_t.h"


drwn::go_to_t* drwn::go_to_t::get_instance() {
    static go_to_t instance;
    return &instance;
}

bool drwn::go_to_t::is_done() const {
    return m_done;
}

void drwn::go_to_t::process(drwn::pose2d_t pos) {
    if (m_debug) {
        LOG_DEBUG << "GO TO: Processing...";
        LOG_DEBUG << "GO TO: pos = ("
                  << pos.get_x() << ", "
                  << pos.get_y() << ", "
                  << degrees(pos.get_theta()) << ')';
    }
    m_done = true;
    float dist = std::hypot(pos.get_x(), pos.get_y());
    float angle = degrees(std::atan2(pos.get_y(), pos.get_x())); // It used when robot goes with turning
    pose2d_t odo = walking_t::get_instance()->get_odo();

    if (!walking_t::get_instance()->is_running() ||
        walking_t::get_instance()->get_x_move_amplitude() != m_x ||
        walking_t::get_instance()->get_x_move_amplitude() != m_y ||
        walking_t::get_instance()->get_x_move_amplitude() != m_a) {
        m_x = walking_t::get_instance()->get_x_move_amplitude();
        m_y = walking_t::get_instance()->get_y_move_amplitude();
        m_a = walking_t::get_instance()->get_a_move_amplitude();
    }

    if (dist > m_distance_var) {
        float angle_diff = angle - odo.get_theta();
        m_goal_max_speed = std::max(std::cos(angle_diff), 0.0f) * (dist < m_fit_distance ? m_fit_speed : m_max_speed);
        float dir = (std::fabs(angle_diff) > m_angle_var ? 1.0f : -1.0f);
        // Reduce x and y and increase a when diff greater than allowable angle variance
        m_x += dir * (m_x > 0 ? -m_step_accel : m_step_accel);
        m_y += 0.0f;
        m_a += dir * (angle_diff > 0.0 ? m_turn_accel : -m_turn_accel);
        // Reset speed when it very small
        float x_abs = std::fabs(m_x);
        if (x_abs < m_step_accel * 0.95f) {
            m_x = 0.0;
        } else if (x_abs > m_goal_max_speed) {
            m_x = std::copysign(m_goal_max_speed, m_x);
        }

        float a_abs = std::fabs(m_a);
        if (a_abs < m_step_accel * 0.95f) {
            m_a = 0.0;
        } else if (a_abs > m_max_turn) {
            m_a = std::copysign(m_max_turn, m_a);
        }

        m_done = false;
    } else {
        m_x = 0;
        m_y = 0;

        float deg = degrees(pos.get_theta());
        if ((deg > 0 && deg > m_angle_var) || (deg < 0 && deg < -m_angle_var)) {
            m_goal_turn = m_max_turn;
            if (deg < 0) {
                m_a -= m_turn_accel;
                if (m_a < -m_goal_turn) m_a = -m_goal_turn;
            } else {
                m_a += m_turn_accel;
                if (m_a > m_goal_turn) m_a = m_goal_turn;
            }
            m_done = false;
        } else {
            m_a = 0;
        }
    }


    if (!m_done) {
        m_update_rate.update();
        walking_t::get_instance()->joint.set_enable_body_without_head(true, true);
        walking_t::get_instance()->set_x_move_amplitude(m_x);
        walking_t::get_instance()->set_y_move_amplitude(m_y);
        walking_t::get_instance()->set_a_move_amplitude(m_a);
        walking_t::get_instance()->start();
    } else {
        walking_t::get_instance()->stop();
    }
}

float drwn::go_to_t::get_max_speed() const {
    return m_max_speed;
}

void drwn::go_to_t::set_max_speed(float max_speed) {
    m_max_speed = max_speed;
}

float drwn::go_to_t::get_fit_speed() const {
    return m_fit_speed;
}

void drwn::go_to_t::set_fit_speed(float fit_speed) {
    m_fit_speed = fit_speed;
}

float drwn::go_to_t::get_max_turn() const {
    return m_max_turn;
}

void drwn::go_to_t::set_max_turn(float max_turn) {
    m_max_turn = max_turn;
}

float drwn::go_to_t::get_step_accel() const {
    return m_step_accel;
}

void drwn::go_to_t::set_step_accel(float step_accel) {
    m_step_accel = step_accel;
}

float drwn::go_to_t::get_turn_accel() const {
    return m_turn_accel;
}

void drwn::go_to_t::set_turn_accel(float turn_accel) {
    m_turn_accel = turn_accel;
}

float drwn::go_to_t::get_fit_distance() const {
    return m_fit_distance;
}

void drwn::go_to_t::set_fit_distance(float fit_distance) {
    m_fit_distance = fit_distance;
}

float drwn::go_to_t::get_distance_var() const {
    return m_distance_var;
}

void drwn::go_to_t::set_distance_var(float distance_var) {
    m_distance_var = distance_var;
}

float drwn::go_to_t::get_angle_var() const {
    return m_angle_var;
}

void drwn::go_to_t::set_angle_var(float angle_var) {
    m_angle_var = angle_var;
}

bool drwn::go_to_t::is_debug_enabled() const {
    return m_debug;
}

void drwn::go_to_t::enable_debug(bool debug) {
    m_debug = debug;
}

std::chrono::duration<int64_t, std::nano> drwn::go_to_t::get_update_rate() const {
    return m_update_rate.get_duration();
}

void drwn::go_to_t::set_update_rate(const std::chrono::duration<int64_t, std::nano>& duration) {
    m_update_rate.set_duration(duration);
}
