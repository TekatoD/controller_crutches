/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "motion/odometry_collector_t.h"
#include <math.h>

drwn::odometry_collector_t::odometry_collector_t()
        : m_initial(0, 0, 0), m_pose(0, 0, 0) { }

drwn::odometry_collector_t::odometry_collector_t(pose2d_t initial)
        : m_initial(initial), m_pose(initial) { }

drwn::odometry_collector_t::odometry_collector_t(float x, float y, float theta)
        : m_initial(x, y, theta), m_pose(x, y, theta) { }

void drwn::odometry_collector_t::odo_translate(pose2d_t offset) {
    float dst = hypot(offset.get_x(), offset.get_y());
    float angle = atan2(offset.get_y(), offset.get_x());
    m_pose.set_x(m_pose.get_x() + (cos(m_pose.get_theta() + angle) * dst));
    m_pose.set_y(m_pose.get_y() + (sin(m_pose.get_theta() + angle + offset.get_theta()) * dst));
    m_pose.set_theta(m_pose.get_theta() + offset.get_theta());
    m_pose.normalize_theta();
}

drwn::pose2d_t drwn::odometry_collector_t::get_pose() const {
    return m_pose;
}

void drwn::odometry_collector_t::set_pose(drwn::pose2d_t offset) {
    m_pose = offset;
}

void drwn::odometry_collector_t::set_initial(drwn::pose2d_t offset) {
    m_initial = offset;
}

void drwn::odometry_collector_t::reset() {
    m_pose = m_initial;
}
