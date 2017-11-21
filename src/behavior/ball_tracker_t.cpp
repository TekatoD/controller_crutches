/*
 *   BallTracker.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <log/trivial_logger_t.h>
#include "motion/modules/head_t.h"
#include "hw/camera_t.h"
#include "behavior/ball_tracker_t.h"

using namespace drwn;

void ball_tracker_t::process(point2d_t pos) {
    if (m_debug) {
        LOG_DEBUG << "BALL TRACKER: Processing has been started";
        LOG_DEBUG << "BALL TRACKER: pos = (" << pos.X << ", " << pos.Y << ')';
    }

    if (pos.X < 0 || pos.Y < 0) {
        m_ball_position.X = -1;
        m_ball_position.Y = -1;
        if (m_no_ball_count < m_no_ball_max_count) {
            head_t::get_instance()->move_tracking();
            m_no_ball_count++;
        }
    } else {
        m_no_ball_count = 0;
        point2d_t center = point2d_t(camera_t::WIDTH / 2, camera_t::HEIGHT / 2);
        point2d_t offset = pos - center;
        offset *= -1; // Inverse X-axis, Y-axis
        offset.X *= (camera_t::VIEW_H_ANGLE / camera_t::WIDTH); // pixel per angle
        offset.Y *= (camera_t::VIEW_V_ANGLE / camera_t::HEIGHT); // pixel per angle
        m_ball_position = offset;
        head_t::get_instance()->move_tracking(m_ball_position);
    }
}

const point2d_t& ball_tracker_t::get_ball_position() const noexcept {
    return m_ball_position;
}

bool ball_tracker_t::is_no_ball() const noexcept {
    return m_no_ball_count >= m_no_ball_max_count;
}

int ball_tracker_t::get_no_ball_max_count() const noexcept {
    return m_no_ball_max_count;
}

void ball_tracker_t::set_no_ball_max_count(int no_ball_max_count) {
    if(m_debug) {
        LOG_DEBUG << "BALL TRACKER: no_ball_max_count = " << no_ball_max_count;
    }
    m_no_ball_max_count = no_ball_max_count;
}

bool ball_tracker_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void ball_tracker_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

ball_tracker_t* ball_tracker_t::get_instance() {
    static ball_tracker_t instance;
    return &instance;
}
