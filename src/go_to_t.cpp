/**
 * Copyright 2016 Arseniy Ivin <arssivka@yandex.ru>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  @autor arssivka
 *  @date 5/3/17
 */

#include "go_to_t.h"
#include "motion/modules/walking_t.h"

#define PI (3.14159265)


bool drwn::go_to_t::is_done() const {
    return m_done;
}

void drwn::go_to_t::process(drwn::pose_2D_t pos) {
    m_done = true;
    float dist = hypot(pos.x(), pos.y());
    float angle = atan2(pos.y(), pos.x()) / M_PI * 180.0;

    if (!walking_t::GetInstance()->is_running() ||
            walking_t::GetInstance()->get_x_move_amplitude() != m_x ||
            walking_t::GetInstance()->get_x_move_amplitude() != m_y ||
            walking_t::GetInstance()->get_x_move_amplitude() != m_a) {
        m_x = walking_t::GetInstance()->get_x_move_amplitude();
        m_y = walking_t::GetInstance()->get_y_move_amplitude();
        m_a = walking_t::GetInstance()->get_a_move_amplitude();
    }


    if (dist > m_distance_var) {
        m_a = 0;
        m_goal_max_speed = (dist < m_fit_distance) ? m_fit_speed : m_max_speed;

        float x_factor = pos.x() / dist;
        float x_speed = x_factor * m_goal_max_speed;
        float y_factor = pos.y() / dist;
        float y_speed = y_factor * m_goal_max_speed;

        m_x += m_step_accel * x_factor;
        if (x_speed > 0 && m_x > x_speed || x_speed <= 0 && m_x < x_speed) {
            m_x = x_speed;
        }

        m_y += m_step_accel * y_factor;
        if (y_speed > 0 && m_y > y_speed || y_speed <= 0 && m_y < y_speed) {
            m_y = y_speed;
        }

        m_done = false;
    } else {
        m_x = 0;
        m_y = 0;
    }

    if (m_done) {
        float deg = pos.theta() / PI * 180;
        if (deg > 0 && deg > m_angle_var || deg < 0 && deg < -m_angle_var) {
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
        walking_t::GetInstance()->joint.set_enable_body_without_head(true, true);
        walking_t::GetInstance()->set_x_move_amplitude(m_x);
        walking_t::GetInstance()->set_y_move_amplitude(m_y);
        walking_t::GetInstance()->set_a_move_amplitude(m_a);
        walking_t::GetInstance()->start();
    } else {
        walking_t::GetInstance()->stop();
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
