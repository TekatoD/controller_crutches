/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "OdometryCollector.h"
#include <math.h>

drwn::OdometryCollector::OdometryCollector()
        : m_initial(0, 0, 0), m_pose(0, 0, 0) { }

drwn::OdometryCollector::OdometryCollector(Pose2D initial)
        : m_initial(initial), m_pose(initial) { }

drwn::OdometryCollector::OdometryCollector(float x, float y, float theta)
        : m_initial(x, y, theta), m_pose(x, y, theta) { }

void drwn::OdometryCollector::odoTranslate(Pose2D offset) {
    float dst = hypot(offset.X(), offset.Y());
    float angle = atan2(offset.Y(), offset.X());
    m_pose.setX(m_pose.X() + (cos(m_pose.Theta() + angle) * dst));
    m_pose.setY(m_pose.Y() + (sin(m_pose.Theta() + angle + offset.Theta()) * dst));
    m_pose.setTheta(m_pose.Theta() + offset.Theta());
    m_pose.normalizeTheta();
}

drwn::Pose2D drwn::OdometryCollector::GetPose() const {
    return m_pose;
}

void drwn::OdometryCollector::SetPose(drwn::Pose2D offset) {
    m_pose = offset;
}

void drwn::OdometryCollector::SetInitial(drwn::Pose2D offset) {
    m_initial = offset;
}

void drwn::OdometryCollector::Reset() {
    m_pose = m_initial;
}
