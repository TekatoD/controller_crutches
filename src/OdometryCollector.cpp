/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "OdometryCollector.h"
#include <math.h>

Robot::OdometryCollector::OdometryCollector() : m_pose(0, 0, 0) { }

Robot::OdometryCollector::OdometryCollector(OdoData initial) : m_pose(initial) { }

Robot::OdometryCollector::OdometryCollector(double x, double y, double theta) : m_pose(x, y, theta) { }

void Robot::OdometryCollector::odoTranslate(OdoData offset) {
    double dst = hypot(offset.getX(), offset.getY());
    double angle = atan2(offset.getY(), offset.getX());
    m_pose.setX(m_pose.getX() + (cos(m_pose.getTheta() + angle) * dst));
    m_pose.setY(m_pose.getY() + (sin(m_pose.getTheta() + angle + offset.getTheta()) * dst));
    m_pose.setTheta(m_pose.getTheta() + offset.getTheta());
    m_pose.normalizeTheta();
}

Robot::OdoData Robot::OdometryCollector::getPose() const {
    return m_pose;
}
