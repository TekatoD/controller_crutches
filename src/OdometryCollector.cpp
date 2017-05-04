/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "OdometryCollector.h"
#include <math.h>

Robot::OdometryCollector::OdometryCollector() : m_initial(0, 0, 0), m_pose(0, 0, 0) { }

Robot::OdometryCollector::OdometryCollector(Pose2D initial) : m_initial(initial), m_pose(initial) { }

Robot::OdometryCollector::OdometryCollector(double x, double y, double theta) : m_initial(x, y, theta), m_pose(x, y, theta) { }

void Robot::OdometryCollector::odoTranslate(Pose2D offset) {
    double dst = hypot(offset.getX(), offset.getY());
    double angle = atan2(offset.getY(), offset.getX());
    m_pose.setX(m_pose.getX() + (cos(m_pose.getTheta() + angle) * dst));
    m_pose.setY(m_pose.getY() + (sin(m_pose.getTheta() + angle + offset.getTheta()) * dst));
    m_pose.setTheta(m_pose.getTheta() + offset.getTheta());
    m_pose.normalizeThetaInPi();
}

Robot::Pose2D Robot::OdometryCollector::GetPose() const {
    return m_pose;
}

void Robot::OdometryCollector::SetPose(Robot::Pose2D offset) {
    m_pose = offset;
}

void Robot::OdometryCollector::SetInitial(Robot::Pose2D offset) {
    m_initial = offset;
}

void Robot::OdometryCollector::Reset() {
    m_pose = m_initial;
}
