/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <iostream>
#include "hw/MX28.h"
#include "motion/modules/Head.h"
#include "motion/modules/Walking.h"
#include "BallFollower.h"
#include "motion/MotionStatus.h"


using namespace Robot;


BallFollower::BallFollower() {
    m_NoBallMaxCount = 10;
    m_NoBallCount = m_NoBallMaxCount;
    m_KickBallMaxCount = 10;
    m_KickBallCount = 0;

    m_KickTopAngle = -5.0;
    m_KickRightAngle = -30.0;
    m_KickLeftAngle = 30.0;

    m_FollowMaxFBStep = 30.0;
    m_FollowMinFBStep = 5.0;
    m_FollowMaxRLTurn = 35.0;
    m_FitFBStep = 3.0;
    m_FitMaxRLTurn = 35.0;
    m_UnitFBStep = 0.3;
    m_UnitRLStep = 0.3;
    m_UnitRLTurn = 1.0;

    m_GoalFBStep = 0;
    m_GoalRLStep = 0;
    m_GoalRLTurn = 0;
    m_FBStep = 0;
    m_RLStep = 0;
    m_RLTurn = 0;
    m_TiltOffset = MX28::RATIO_VALUE2DEGREES;
    m_KickBall = NO_KICKING;

    m_AimTiltOffset = 15;
    m_AimRLTurn = 10;
    m_AimRLStep = 20;
}


BallFollower::~BallFollower() {
}


void BallFollower::Process(Point2D ball_pos,
                           float angle_top,
                           float angle_bot) {
    bool aim = false;

    if (ball_pos.X == -1.0 || ball_pos.Y == -1.0) {
        m_KickBall = NO_KICKING;

        if (m_NoBallCount > m_NoBallMaxCount) {
            // can not find a ball
            m_GoalFBStep = 0;
            m_GoalRLTurn = 0;
            Head::GetInstance()->MoveToHome();
        } else {
            m_NoBallCount++;
        }
    } else {
        m_NoBallCount = 0;

        float pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        float pan_range = Head::GetInstance()->GetLeftLimitAngle();
        float pan_percent = pan / pan_range;

        float tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
        float tilt_min = Head::GetInstance()->GetBottomLimitAngle();
        float tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
        float tilt_percent = (tilt - tilt_min) / tilt_range;
        if (tilt_percent < 0)
            tilt_percent = -tilt_percent;
        if (pan > m_KickRightAngle && pan < m_KickLeftAngle) {
            if (tilt <= (tilt_min + m_AimTiltOffset) && (angle_top > 0 || angle_bot < 0)) {
                m_KickBallCount = 0;
                aim = true;
                m_GoalFBStep = 0;
                if (angle_top > 0) {
                    m_GoalRLStep = -m_AimRLStep;
                    m_GoalRLTurn = m_AimRLTurn * pan_percent;
                } else {
                    m_GoalRLStep = m_AimRLStep;
                    m_GoalRLTurn = -m_AimRLTurn * pan_percent;
                }
            } else if (tilt <= (tilt_min + m_TiltOffset)) {
                if (ball_pos.Y < m_KickTopAngle) {
                    m_GoalFBStep = 0;
                    m_GoalRLStep = 0;
                    m_GoalRLTurn = 0;

                    if (m_KickBallCount >= m_KickBallMaxCount) {
                        m_FBStep = 0;
                        m_RLStep = 0;
                        m_RLTurn = 0;
                        if (pan > 0) {
                            m_KickBall = LEFT_LEG_KICK; // Left
                        } else {
                            m_KickBall = RIGHT_LEG_KICK; // Right
                        }
                    } else {
                        m_KickBall = NO_KICKING;
                    }
                } else {
                    m_KickBallCount = 0;
                    m_KickBall = NO_KICKING;
                    m_GoalFBStep = m_FitFBStep;
                    m_GoalRLStep = 0;
                    m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
                }
            } else {
                m_KickBallCount = 0;
                m_KickBall = NO_KICKING;
                m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
                if (m_GoalFBStep < m_FollowMinFBStep)
                    m_GoalFBStep = m_FollowMinFBStep;
                m_GoalRLStep = 0;
                m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
            }
        } else {
            m_KickBallCount = 0;
            m_KickBall = NO_KICKING;
            m_GoalFBStep = 0;
            m_GoalRLStep = 0;
            m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
        }
    }

    if (m_GoalFBStep == 0 && m_GoalRLStep == 0 && m_GoalRLStep == 0 &&
            m_FBStep == 0 && m_RLTurn == 0 && m_RLStep == 0) {
        if (Walking::GetInstance()->IsRunning()) {
            Walking::GetInstance()->Stop();
        } else {
            if (m_KickBallCount < m_KickBallMaxCount)
                m_KickBallCount++;
        }
    } else {
        if (!Walking::GetInstance()->IsRunning()) {

            m_FBStep = 0;
            m_RLStep = 0;
            m_RLTurn = 0;
            m_KickBallCount = 0;
            m_KickBall = NO_KICKING;
            
            Walking::GetInstance()->SetXMoveAmplitude(m_FBStep);
            Walking::GetInstance()->SetYMoveAmplitude(m_RLStep);
            Walking::GetInstance()->SetAMoveAmplitude(m_RLTurn);
            Walking::GetInstance()->Start();
        } else {
            if (m_FBStep < m_GoalFBStep)
                m_FBStep += m_UnitFBStep;
            else if (m_FBStep > m_GoalFBStep)
                m_FBStep = m_GoalFBStep;
            Walking::GetInstance()->SetXMoveAmplitude(m_FBStep);

            if (m_GoalRLStep > 0) {
                if (m_RLStep < m_GoalRLStep)
                    m_RLStep += m_UnitRLStep;
                else if (m_FBStep > m_GoalRLStep)
                    m_RLStep = m_GoalRLStep;
            } else {
                if (m_RLStep > -m_GoalRLStep)
                    m_RLStep -= m_UnitRLStep;
                else if (m_FBStep < -m_GoalRLStep)
                    m_RLStep = -m_GoalRLStep;
            }
            Walking::GetInstance()->SetYMoveAmplitude(m_RLStep);

            if (m_RLTurn < m_GoalRLTurn)
                m_RLTurn += m_UnitRLTurn;
            else if (m_RLTurn > m_GoalRLTurn)
                m_RLTurn -= m_UnitRLTurn;
            Walking::GetInstance()->SetAMoveAmplitude(m_RLTurn);
            Walking::GetInstance()->SetMoveAimOn(aim);
        }
    }
}

bool BallFollower::IsNoBall() const {
    return m_NoBallCount >= m_NoBallCount;
}

KickingAction BallFollower::GetKickingLeg() const {
    return m_KickBall;
}
