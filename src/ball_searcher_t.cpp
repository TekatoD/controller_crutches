/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#include <hw/camera_t.h>
#include <motion/modules/head_t.h>
#include <cmath>
#include <motion/modules/walking_t.h>
#include <log/trivial_logger_t.h>
#include "ball_searcher_t.h"

using namespace drwn;

ball_searcher_t::ball_searcher_t() {
    m_active = false;

    m_tilt_phase = 0.0;
    m_pan_phase = 0.0;
    m_tilt_phase_step = 1.0;
    m_pan_phase_step = 2.0;
    m_phase_size = 500.0;

    m_turn_speed = 0.0;
    m_turn_step = 1.0;
    m_max_turn = 20.0;

    m_turn_direction = 0;
    m_pan_direction = 0;

    m_walking_enabled = true;
    m_last_position = point_2D_t(camera_t::WIDTH / 2, camera_t::HEIGHT / 2);
}

void ball_searcher_t::process() {
   if (!m_active) {
       // TODO Pan checking
        point_2D_t center = point_2D_t(camera_t::WIDTH / 2, camera_t::HEIGHT / 2);
        point_2D_t offset = m_last_position - center;

        m_pan_phase = 0.0;
        m_tilt_phase = 0.0;

        m_pan_direction = offset.X > 0 ? 1 : -1;
        m_turn_direction = offset.Y > 0 ? 1 : -1;

        m_active = true;
    }


    m_pan_phase += m_pan_phase_step * m_pan_direction;
    m_tilt_phase += m_tilt_phase_step * m_turn_direction;

    const float tilt_max = head_t::GetInstance()->get_top_limit_angle();
    const float tilt_min = head_t::GetInstance()->get_bottom_limit_angle();
    const float tilt_diff = tilt_max - tilt_min;
    const float pan_max = head_t::GetInstance()->get_left_limit_angle();
    const float pan_min = head_t::GetInstance()->get_right_limit_angle();
    const float pan_diff = pan_max - pan_min;



    float pan = sinf(m_pan_phase / m_phase_size * M_2_PI) * pan_diff - pan_min;
    float tilt = sinf(m_tilt_phase / m_phase_size * M_2_PI) * tilt_diff - tilt_min;
    head_t::GetInstance()->move_by_angle(pan, tilt);

    if (m_walking_enabled) {
        m_turn_speed = walking_t::GetInstance()->get_a_move_amplitude();
        m_turn_speed += m_turn_step * m_turn_direction;
        if (fabsf(m_turn_speed) > m_max_turn) {
            m_turn_speed = m_max_turn * m_turn_direction;
        }

        walking_t::GetInstance()->set_x_move_amplitude(0);
        walking_t::GetInstance()->set_x_move_amplitude(0);
        walking_t::GetInstance()->set_x_move_amplitude(m_turn_speed);
        walking_t::GetInstance()->set_move_aim_on(false);
        walking_t::GetInstance()->start();
    } else {
        walking_t::GetInstance()->stop();
    }
}

void ball_searcher_t::enable_walking() {
    m_walking_enabled = true;
}

void ball_searcher_t::disable_walking() {
    m_walking_enabled = false;
}

bool ball_searcher_t::is_walking_enabled() const {
    return m_walking_enabled;
}

const point_2D_t& ball_searcher_t::get_last_position() const {
    return m_last_position;
}

void ball_searcher_t::set_last_position(const point_2D_t& pos) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: last_position_x = " << pos.X << " last_position_y = " << pos.Y;
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
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: tilt_phase_step = " << tilt_phase_step;
    }
    m_tilt_phase_step = tilt_phase_step;
}

float ball_searcher_t::get_pan_phase_step() const {
    return m_pan_phase_step;
}

void ball_searcher_t::set_pan_phase_step(float pan_phase_step) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: pan_phase_step = " << pan_phase_step;
    }
    m_pan_phase_step = pan_phase_step;
}

float ball_searcher_t::get_phase_size() const {
    return m_phase_size;
}

void ball_searcher_t::set_phase_size(float phase_size) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: phase_size = " << phase_size;
    }
    m_phase_size = phase_size;
}

float ball_searcher_t::get_turn_step() const {
    return m_turn_step;
}

void ball_searcher_t::set_turn_step(float turn_step) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: turn_step = " << turn_step;
    }
    m_turn_step = turn_step;
}

float ball_searcher_t::get_max_turn() const {
    return m_max_turn;
}

void ball_searcher_t::set_max_turn(float max_turn) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: max_turn = " << max_turn;
    }
    m_max_turn = max_turn;
}

void ball_searcher_t::enable_debug(bool debug) { m_debug = debug; }

bool ball_searcher_t::is_debug_enabled() const { return m_debug; }
