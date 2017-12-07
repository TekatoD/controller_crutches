/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "motion/odometry_collector_t.h"
#include <cmath>
#include <log/trivial_logger_t.h>
#include <math/angle_tools.h>

using namespace drwn;

odometry_collector_t::odometry_collector_t(const pose2d_t& initial)
        : m_pose(initial) { }

odometry_collector_t::odometry_collector_t(float x, float y, float theta)
        : m_pose(x, y, theta) { }

void odometry_collector_t::odo_translate(const pose2d_t& offset) {
    float dst = std::hypot(offset.get_x(), offset.get_y());
    float angle = std::atan2(offset.get_y(), offset.get_x());
    m_pose.set_x(m_pose.get_x() + (std::cos(m_pose.get_theta() + angle) * dst));
    m_pose.set_y(m_pose.get_y() + (std::sin(m_pose.get_theta() + angle + offset.get_theta()) * dst));
    m_pose.set_theta(m_pose.get_theta() + offset.get_theta());
    m_pose.normalize_theta();
    if (m_debug) LOG_DEBUG << "ODOMETRY COLLECTOR: translated pose = ("
                           << m_pose.get_x() << ", "
                           << m_pose.get_y() << ", "
                           << degrees(m_pose.get_theta()) << ')';
}

const pose2d_t& odometry_collector_t::get_pose() const {
    return m_pose;
}

void odometry_collector_t::set_pose(const pose2d_t& offset) {
    if (m_debug && offset != m_pose) LOG_DEBUG << "ODOMETRY COLLECTOR: pose = ("
                           << offset.get_x() << ", "
                           << offset.get_y() << ", "
                           << degrees(offset.get_theta()) << ')';
    m_pose = offset;
}

bool odometry_collector_t::is_debug_enabled() const {
    return m_debug;
}

void odometry_collector_t::enable_debug(bool debug) {
    m_debug = debug;
}
