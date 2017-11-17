/**
 *  @autor arssivka
 *  @date 5/3/17
 */

#include <math/angle_tools.h>
#include "behavior/go_to_t.h"
#include "motion/modules/walking_t.h"


bool drwn::go_to_t::is_done() const {
    return m_done;
}

void drwn::go_to_t::process(drwn::pose2d_t pos) {
    m_done = true;
    float dist = hypotf(pos.x(), pos.y());
    float angle = degrees(atan2f(pos.y(), pos.x()));

    if (!walking_t::get_instance()->is_running() ||
            walking_t::get_instance()->get_x_move_amplitude() != m_x ||
            walking_t::get_instance()->get_x_move_amplitude() != m_y ||
            walking_t::get_instance()->get_x_move_amplitude() != m_a) {
        m_x = walking_t::get_instance()->get_x_move_amplitude();
        m_y = walking_t::get_instance()->get_y_move_amplitude();
        m_a = walking_t::get_instance()->get_a_move_amplitude();
    }


    if (dist > m_distance_var) {
        m_a = 0;
        m_goal_max_speed = (dist < m_fit_distance) ? m_fit_speed : m_max_speed;

        float x_factor = pos.x() / dist;
        float x_speed = x_factor * m_goal_max_speed;
        float y_factor = pos.y() / dist;
        float y_speed = y_factor * m_goal_max_speed;

        m_x += m_step_accel * x_factor;
        if ((x_speed > 0 && m_x > x_speed) || (x_speed <= 0 && m_x < x_speed)) {
            m_x = x_speed;
        }

        m_y += m_step_accel * y_factor;
        if ((y_speed > 0 && m_y > y_speed) || (y_speed <= 0 && m_y < y_speed)) {
            m_y = y_speed;
        }

        m_done = false;
    } else {
        m_x = 0;
        m_y = 0;
    }

    if (m_done) {
        float deg = degrees(pos.theta());
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
        walking_t::get_instance()->joint.set_enable_body_without_head(true, true);
        walking_t::get_instance()->set_x_move_amplitude(m_x);
        walking_t::get_instance()->set_y_move_amplitude(m_y);
        walking_t::get_instance()->set_a_move_amplitude(m_a);
        walking_t::get_instance()->start();
    } else {
        walking_t::get_instance()->stop();
    }
}

drwn::go_to_t::go_to_t() {
    m_max_speed = 20.0;
    m_fit_speed = 3.0;
    m_max_turn = 35.0;
    m_step_accel = 1.0;
    m_turn_accel = 1.0;

    m_fit_distance = 200.0;
    m_distance_var = 50.0;
    m_angle_var = 10.0;

    m_goal_max_speed = 0.0;
    m_goal_turn = 0.0;

    m_x = 0.0;
    m_y = 0.0;
    m_a = 0.0;

    m_done = false;
}
