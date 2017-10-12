/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <iostream>
#include "ImgProcess.h"
#include "MX28.h"
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


void BallFollower::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, FOLLOWER_SECTION);
}


void BallFollower::LoadINISettings(minIni* ini, const std::string& section) {
    int value = -2;
    if ((value = ini->geti(section, "no_ball_max_count", INVALID_VALUE)) != INVALID_VALUE) m_NoBallMaxCount = value;
    if ((value = ini->geti(section, "kick_ball_max_count", INVALID_VALUE)) != INVALID_VALUE) m_KickBallMaxCount = value;

    float dvalue = -2.0;
    if ((dvalue = ini->getd(section, "kick_top_angle", INVALID_VALUE)) != INVALID_VALUE) m_KickTopAngle = dvalue;
    if ((dvalue = ini->getd(section, "kick_right_angle", INVALID_VALUE)) != INVALID_VALUE) m_KickRightAngle = dvalue;
    if ((dvalue = ini->getd(section, "kick_left_angle", INVALID_VALUE)) != INVALID_VALUE) m_KickLeftAngle = dvalue;
    if ((dvalue = ini->getd(section, "follow_max_fb_step", INVALID_VALUE)) != INVALID_VALUE) m_FollowMaxFBStep = dvalue;
    if ((dvalue = ini->getd(section, "follow_min_fb_step", INVALID_VALUE)) != INVALID_VALUE) m_FollowMinFBStep = dvalue;
    if ((dvalue = ini->getd(section, "follow_max_rl_turn", INVALID_VALUE)) != INVALID_VALUE) m_FollowMaxRLTurn = dvalue;
    if ((dvalue = ini->getd(section, "fit_fb_step", INVALID_VALUE)) != INVALID_VALUE) m_FitFBStep = dvalue;
    if ((dvalue = ini->getd(section, "fit_max_rl_turn", INVALID_VALUE)) != INVALID_VALUE) m_FitMaxRLTurn = dvalue;
    if ((dvalue = ini->getd(section, "unit_fb_step", INVALID_VALUE)) != INVALID_VALUE) m_UnitFBStep = dvalue;
    if ((dvalue = ini->getd(section, "unit_rl_step", INVALID_VALUE)) != INVALID_VALUE) m_UnitRLStep = dvalue;
    if ((dvalue = ini->getd(section, "unit_rl_turn", INVALID_VALUE)) != INVALID_VALUE) m_UnitRLTurn = dvalue;
    if ((dvalue = ini->getd(section, "tilt_offset", INVALID_VALUE)) != INVALID_VALUE) m_TiltOffset = dvalue;
    if ((dvalue = ini->getd(section, "aim_tilt_offset", INVALID_VALUE)) != INVALID_VALUE) m_AimTiltOffset = dvalue;
    if ((dvalue = ini->getd(section, "aim_rl_step", INVALID_VALUE)) != INVALID_VALUE) m_AimRLStep = dvalue;
    if ((dvalue = ini->getd(section, "aim_rl_turn", INVALID_VALUE)) != INVALID_VALUE) m_AimRLTurn = dvalue;
}


void BallFollower::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, FOLLOWER_SECTION);
}


void BallFollower::SaveINISettings(minIni* ini, const std::string& section) {
    ini->put(section, "no_ball_max_count", m_NoBallMaxCount);
    ini->put(section, "kick_ball_max_count", m_KickBallMaxCount);
    ini->put(section, "kick_top_angle", m_KickTopAngle);
    ini->put(section, "kick_right_angle", m_KickRightAngle);
    ini->put(section, "kick_left_angle", m_KickLeftAngle);
    ini->put(section, "follow_max_fb_step", m_FollowMaxFBStep);
    ini->put(section, "follow_min_fb_step", m_FollowMinFBStep);
    ini->put(section, "follow_max_rl_turn", m_FollowMaxRLTurn);
    ini->put(section, "fit_fb_step", m_FitFBStep);
    ini->put(section, "fit_max_rl_turn", m_FitMaxRLTurn);
    ini->put(section, "unit_fb_step", m_UnitFBStep);
    ini->put(section, "unit_rl_step", m_UnitRLStep);
    ini->put(section, "unit_rl_turn", m_UnitRLTurn);
    ini->put(section, "tilt_offset", m_TiltOffset);
    ini->put(section, "aim_tilt_offset", m_AimTiltOffset);
    ini->put(section, "aim_rl_step", m_AimRLStep);
    ini->put(section, "aim_rl_turn", m_AimRLTurn);
}

bool BallFollower::IsNoBall() const {
    return m_NoBallCount >= m_NoBallCount;
}

KickingAction BallFollower::GetKickingLeg() const {
    return m_KickBall;
}
