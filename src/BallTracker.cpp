/*
 *   BallTracker.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "motion/modules/Head.h"
#include "Camera.h"
#include "BallTracker.h"

using namespace Robot;


BallTracker::BallTracker()
        : BallPosition(Point2D(-1.0, -1.0)) {
    m_NoBallCount = 0;
    m_NoBallMaxCount = 15;
}


BallTracker::~BallTracker() {
}


void BallTracker::Process(Point2D pos) {
    if (pos.X < 0 || pos.Y < 0) {
        BallPosition.X = -1;
        BallPosition.Y = -1;
        if (m_NoBallCount < m_NoBallMaxCount) {
            Head::GetInstance()->MoveTracking();
            m_NoBallCount++;
        }
//        else
//            Head::GetInstance()->InitTracking();
    } else {
        m_NoBallCount = 0;
        Point2D center = Point2D(Camera::WIDTH / 2, Camera::HEIGHT / 2);
        Point2D offset = pos - center;
        offset *= -1; // Inverse X-axis, Y-axis
        offset.X *= (Camera::VIEW_H_ANGLE / (float) Camera::WIDTH); // pixel per angle
        offset.Y *= (Camera::VIEW_V_ANGLE / (float) Camera::HEIGHT); // pixel per angle
        BallPosition = offset;
        Head::GetInstance()->MoveTracking(BallPosition);
    }
}

const Point2D& BallTracker::GetBallPosition() const {
    return BallPosition;
}

bool BallTracker::IsNoBall() const {
    return m_NoBallCount >= m_NoBallMaxCount;
}

int BallTracker::GetNoBallMaxCount() const {
    return m_NoBallMaxCount;
}

void BallTracker::SetNoBallMaxCount(int no_ball_max_count) {
    m_NoBallMaxCount = no_ball_max_count;
}
