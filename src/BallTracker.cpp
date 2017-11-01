/*
 *   BallTracker.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <log/trivial_logger_t.h>
#include "motion/modules/head_t.h"
#include "Camera.h"
#include "BallTracker.h"

using namespace drwn;


BallTracker::BallTracker()
        : BallPosition(point_2D_t(-1.0, -1.0)) {
    m_NoBallCount = 0;
    m_NoBallMaxCount = 15;
}


BallTracker::~BallTracker() {
}


void BallTracker::Process(point_2D_t pos) {
    if (pos.X < 0 || pos.Y < 0) {
        BallPosition.X = -1;
        BallPosition.Y = -1;
        if (m_NoBallCount < m_NoBallMaxCount) {
            head_t::GetInstance()->move_tracking();
            m_NoBallCount++;
        }
//        else
//            head_t::GetInstance()->init_tracking();
    } else {
        m_NoBallCount = 0;
        point_2D_t center = point_2D_t(Camera::WIDTH / 2, Camera::HEIGHT / 2);
        point_2D_t offset = pos - center;
        offset *= -1; // Inverse X-axis, Y-axis
        offset.X *= (Camera::VIEW_H_ANGLE / (float) Camera::WIDTH); // pixel per angle
        offset.Y *= (Camera::VIEW_V_ANGLE / (float) Camera::HEIGHT); // pixel per angle
        BallPosition = offset;
        head_t::GetInstance()->move_tracking(BallPosition);
    }
}

const point_2D_t& BallTracker::GetBallPosition() const {
    return BallPosition;
}

bool BallTracker::IsNoBall() const {
    return m_NoBallCount >= m_NoBallMaxCount;
}

int BallTracker::GetNoBallMaxCount() const {
    return m_NoBallMaxCount;
}

void BallTracker::SetNoBallMaxCount(int no_ball_max_count) {
    if(m_debug) {
        LOG_DEBUG << "BALL TRACKER: no_ball_max_count = " << no_ball_max_count;
    }
    m_NoBallMaxCount = no_ball_max_count;
}

bool BallTracker::IsDebugEnabled() const {
    return m_debug;
}

void BallTracker::EnableDebug(bool debug) {
    m_debug = debug;
}
