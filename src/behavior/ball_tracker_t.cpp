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
        m_no_ball = m_no_ball_rate.is_passed();
        if (!m_no_ball) {
            if (m_debug) {
                LOG_DEBUG << "BALL TRACKER: Continues to tracking";
            }
            head_t::get_instance()->move_tracking();
        } else if (m_debug) {
            LOG_DEBUG << "BALL TRACKER: Ball is lost";
        }
    } else {
        if (m_debug) LOG_DEBUG << "BALL TRACKER: Tracking the ball";
        m_no_ball = false;
        point2d_t center(camera_t::WIDTH / 2, camera_t::HEIGHT / 2);
        point2d_t offset = pos - center;
        offset *= -1; // Inverse X-axis, Y-axis
        offset.X *= (camera_t::VIEW_H_ANGLE / camera_t::WIDTH); // pixel per angle
        offset.Y *= (camera_t::VIEW_V_ANGLE / camera_t::HEIGHT); // pixel per angle
        if (m_no_ball_rate.is_passed() ||
                std::hypot(offset.X - m_ball_position.X, offset.Y - m_ball_position.X) <= m_distance_threshold) {
            m_no_ball_rate.update();
            m_ball_position = offset;
            head_t::get_instance()->move_tracking(m_ball_position);
        }
    }
}

const point2d_t& ball_tracker_t::get_ball_position() const noexcept {
    return m_ball_position;
}

bool ball_tracker_t::is_no_ball() const noexcept {
    return m_no_ball;
}

steady_rate_t::duration ball_tracker_t::get_no_ball_duration() const noexcept {
    return m_no_ball_rate.get_duration();
}

void ball_tracker_t::set_no_ball_duration(steady_rate_t::duration duration) {
    using namespace std::chrono;
    if(m_debug) {
        LOG_DEBUG << "BALL TRACKER: no_ball_duration = " << duration_cast<milliseconds>(duration).count();
    }
    m_no_ball_rate.set_duration(duration);
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

float ball_tracker_t::get_distance_threshold() const {
    return m_distance_threshold;
}

void ball_tracker_t::set_distance_threshold(float distance_threshold) {
    if (m_debug) LOG_DEBUG << "BALL TRACKER: distance_threshold = " << distance_threshold;
    m_distance_threshold = distance_threshold;
}
