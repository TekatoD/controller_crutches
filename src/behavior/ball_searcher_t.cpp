/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#include <hw/camera_t.h>
#include <motion/modules/head_t.h>
#include <motion/modules/walking_t.h>
#include <log/trivial_logger_t.h>
#include <boost/math/constants/constants.hpp>
#include "behavior/ball_searcher_t.h"

using namespace drwn;

void ball_searcher_t::process() {
    using namespace boost::math;

    if (m_debug) LOG_DEBUG << "BALL SEARCHER: Processing has been started";

    auto half_phase_size = m_phase_size / 2.0f;
    if (!m_active) {
        // TODO Pan checking
        point2d_t center = point2d_t(camera_t::WIDTH / 2, camera_t::HEIGHT / 2);
        point2d_t offset = m_last_position - center;

        m_pan_phase = half_phase_size;
        m_tilt_phase = half_phase_size;

        m_pan_direction = head_t::get_instance()->get_pan_angle() > 0 ? 1 : -1;
        m_turn_direction = head_t::get_instance()->get_tilt_angle() > 0 ? 1 : -1;

        m_active = true;
    }


    m_pan_phase = std::fmod(m_pan_phase + m_pan_phase_step, m_phase_size);
    m_tilt_phase = std::fmod(m_tilt_phase + m_tilt_phase_step, m_phase_size);
    

    const float tilt_max = head_t::get_instance()->get_top_limit_angle();
    const float tilt_min = head_t::get_instance()->get_bottom_limit_angle();
    const float tilt_diff = tilt_max - tilt_min;
    const float pan_max = head_t::get_instance()->get_left_limit_angle();
    const float pan_min = head_t::get_instance()->get_right_limit_angle();
    const float pan_diff = pan_max - pan_min;

    float pan = 0.0f;
    if (m_pan_enabled) {
        pan = (m_pan_phase - half_phase_size) / half_phase_size * pan_diff / 2.0f * m_pan_direction;
    }
    float tilt = m_tilt_phase <= half_phase_size
                 ? tilt_min + tilt_diff / 2.0f
                 : tilt_min;
    head_t::get_instance()->move_by_angle(pan, tilt);

    if (m_debug) {
        LOG_DEBUG << "BALL SEARCHER: pan_phase = " << m_pan_phase;
        LOG_DEBUG << "BALL SEARCHER: tilt_phase = " << m_tilt_phase;
        LOG_DEBUG << "BALL SEARCHER: pan = " << pan;
        LOG_DEBUG << "BALL SEARCHER: tilt = " << tilt;
    }

    if (m_walking_enabled) {
        m_turn_speed = walking_t::get_instance()->get_a_move_amplitude();
        m_turn_speed += m_turn_step * m_turn_direction;
        if (std::fabs(m_turn_speed) > m_max_turn) {
            m_turn_speed = m_max_turn * m_turn_direction;
        }

        walking_t::get_instance()->set_x_move_amplitude(0);
        walking_t::get_instance()->set_y_move_amplitude(0);
        walking_t::get_instance()->set_a_move_amplitude(m_turn_speed);
        walking_t::get_instance()->set_move_aim_on(false);
        walking_t::get_instance()->start();
    } else {
        walking_t::get_instance()->stop();
    }
}

void ball_searcher_t::enable_walking(bool enabled) {
    if (m_debug) LOG_DEBUG << "BALL SEARCHER: walking_enabled = " << std::boolalpha << enabled;
    m_walking_enabled = enabled;
}

bool ball_searcher_t::is_walking_enabled() const {
    return m_walking_enabled;
}

const point2d_t& ball_searcher_t::get_last_position() const {
    return m_last_position;
}

void ball_searcher_t::set_last_position(const point2d_t& pos) {
    if (m_debug) {
        LOG_DEBUG << "BALL SEARCHER: last_position = (" << pos.X << ", " << pos.Y << ')';
    }
    if (pos.X != -1 && pos.Y != -1) {
        m_last_position = pos;
        m_active = false;
    }
}

float ball_searcher_t::get_tilt_phase_step() const {
    return m_tilt_phase_step;
}

void ball_searcher_t::set_tilt_phase_step(float tilt_phase_step) {
    if (m_debug) {
        LOG_DEBUG << "BALL SEARCHER: tilt_phase_step = " << tilt_phase_step;
    }
    m_tilt_phase_step = tilt_phase_step;
}

float ball_searcher_t::get_pan_phase_step() const {
    return m_pan_phase_step;
}

void ball_searcher_t::set_pan_phase_step(float pan_phase_step) {
    if (m_debug) {
        LOG_DEBUG << "BALL SEARCHER: pan_phase_step = " << pan_phase_step;
    }
    m_pan_phase_step = pan_phase_step;
}

float ball_searcher_t::get_phase_size() const {
    return m_phase_size;
}

void ball_searcher_t::set_phase_size(float phase_size) {
    if (m_debug) {
        LOG_DEBUG << "BALL SEARCHER: phase_size = " << phase_size;
    }
    m_phase_size = phase_size;
}

float ball_searcher_t::get_turn_step() const {
    return m_turn_step;
}

void ball_searcher_t::set_turn_step(float turn_step) {
    if (m_debug) {
        LOG_DEBUG << "BALL SEARCHER: turn_step = " << turn_step;
    }
    m_turn_step = turn_step;
}

float ball_searcher_t::get_max_turn() const {
    return m_max_turn;
}

void ball_searcher_t::set_max_turn(float max_turn) {
    if (m_debug) {
        LOG_DEBUG << "BALL SEARCHER: max_turn = " << max_turn;
    }
    m_max_turn = max_turn;
}

void ball_searcher_t::enable_debug(bool debug) { m_debug = debug; }

bool ball_searcher_t::is_debug_enabled() const { return m_debug; }

ball_searcher_t* ball_searcher_t::get_instance() {
    static ball_searcher_t instance;
    return &instance;
}

bool ball_searcher_t::is_pan_enabled() const {
    return m_pan_enabled;
}

void ball_searcher_t::enable_pan(bool pan_active) {
    m_pan_enabled = pan_active;
}
