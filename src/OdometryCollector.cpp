/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "OdometryCollector.h"
#include <math.h>

Robot::OdometryCollector::OdometryCollector() : m_initial(0, 0, 0), m_pose(0, 0, 0) { }

Robot::OdometryCollector::OdometryCollector(Pose2D initial) : m_initial(initial), m_pose(initial) { }

Robot::OdometryCollector::OdometryCollector(float x, float y, float theta) : m_initial(x, y, theta), m_pose(x, y, theta) { }

void Robot::OdometryCollector::odoTranslate(Pose2D offset) {
    float dst = hypot(offset.X(), offset.Y());
    float angle = atan2(offset.Y(), offset.X());
    m_pose.setX(m_pose.X() + (cos(m_pose.Theta() + angle) * dst));
    m_pose.setY(m_pose.Y() + (sin(m_pose.Theta() + angle + offset.Theta()) * dst));
    m_pose.setTheta(m_pose.Theta() + offset.Theta());
    m_pose.normalizeTheta();
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
