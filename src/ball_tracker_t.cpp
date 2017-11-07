/*
 *   BallTracker.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <log/trivial_logger_t.h>
#include "motion/modules/head_t.h"
#include "hw/camera_t.h"
#include "ball_tracker_t.h"

using namespace drwn;


ball_tracker_t::ball_tracker_t()
        : m_ball_position(point_2D_t(-1.0, -1.0)) {
    m_no_ball_count = 0;
    m_no_ball_max_count = 15;
}


ball_tracker_t::~ball_tracker_t() {
}


void ball_tracker_t::process(point_2D_t pos) {
    if (pos.X < 0 || pos.Y < 0) {
        m_ball_position.X = -1;
        m_ball_position.Y = -1;
        if (m_no_ball_count < m_no_ball_max_count) {
            head_t::get_instance()->move_tracking();
            m_no_ball_count++;
        }
//        else
//            head_t::get_instance()->init_tracking();
    } else {
        m_no_ball_count = 0;
        point_2D_t center = point_2D_t(camera_t::WIDTH / 2, camera_t::HEIGHT / 2);
        point_2D_t offset = pos - center;
        offset *= -1; // Inverse X-axis, Y-axis
        offset.X *= (camera_t::VIEW_H_ANGLE / (float) camera_t::WIDTH); // pixel per angle
        offset.Y *= (camera_t::VIEW_V_ANGLE / (float) camera_t::HEIGHT); // pixel per angle
        m_ball_position = offset;
        head_t::get_instance()->move_tracking(m_ball_position);
    }
}

const point_2D_t& ball_tracker_t::get_ball_position() const {
    return m_ball_position;
}

bool ball_tracker_t::is_no_ball() const {
    return m_no_ball_count >= m_no_ball_max_count;
}

int ball_tracker_t::get_no_ball_max_count() const {
    return m_no_ball_max_count;
}

void ball_tracker_t::set_no_ball_max_count(int no_ball_max_count) {
    if(m_debug) {
        LOG_DEBUG << "BALL TRACKER: no_ball_max_count = " << no_ball_max_count;
    }
    m_no_ball_max_count = no_ball_max_count;
}

bool ball_tracker_t::is_debug_enabled() const {
    return m_debug;
}

void ball_tracker_t::enable_debug(bool debug) {
    m_debug = debug;
}
