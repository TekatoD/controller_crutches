/**
 *  @autor tekatod
 *  @date 5/5/17
 */

#include <stdio.h>
#include <iostream>
#include <motion/modules/Action.h>
#include <LinuxCamera.h>
#include "ImgProcess.h"
#include "MX28.h"
#include "motion/modules/Head.h"
#include "motion/modules/Walking.h"
#include "GoalieBehavior.h"
#include "motion/MotionStatus.h"


using namespace Robot;


GoalieBehavior::GoalieBehavior() {
    m_NoBallMaxCount = 10;
    m_NoBallCount = m_NoBallMaxCount;
    m_KickBallMaxCount = 10;
    m_KickBallCount = 0;

    m_KickTopAngle = -5.0;
    m_KickRightAngle = -30.0;
    m_KickLeftAngle = 30.0;

    m_FollowMaxFBStep = 30.0;
    m_FollowMinFBStep = 5.0;
    m_FollowMaxRLTurn = 30.0;
    m_FitFBStep = 3.0;
    m_FitMaxRLTurn = 30.0;
    m_UnitFBStep = 0.3;
    m_UnitRLTurn = 1.0;

    m_GoalFBStep = 0;
    m_GoalRLTurn = 0;
    m_FBStep = 0;
    m_RLTurn = 0;
    DEBUG_PRINT = false;
    KickBall = 0;
    m_edge_dist_threshhold = 80.0;
    m_follow_threshhold = 2;
    m_right_x = 0;
    m_right_y = 0;
    m_left_x = 0;
    m_left_y = 0;
}


GoalieBehavior::~GoalieBehavior() {
}


void GoalieBehavior::Process() {
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    LinuxCamera::GetInstance()->CaptureFrame();
    m_BallTracker.Process(m_BallFinder.GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

    if (m_BallTracker.ball_position.X == -1.0 || m_BallTracker.ball_position.Y == -1.0) {
        KickBall = 0;

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

        double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        double pan_range = Head::GetInstance()->GetLeftLimitAngle();
        double pan_percent = pan / pan_range;

        double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
        double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
        double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
        double tilt_percent = (tilt - tilt_min) / tilt_range;
        if (tilt_percent < 0)
            tilt_percent = -tilt_percent;

        if (pan > m_KickRightAngle && pan < m_KickLeftAngle) {
            if (tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE)) {
                if (m_BallTracker.ball_position.Y < m_KickTopAngle) {
                    m_GoalFBStep = 0;
                    m_GoalRLTurn = 0;

                    if (m_KickBallCount >= m_KickBallMaxCount) {
                        m_FBStep = 0;
                        m_RLTurn = 0;
                        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                        Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                        if (pan > 0) {
                            KickBall = 1; // Left
                            Action::GetInstance()->Start(13);
                        } else {
                            KickBall = -1; // Right
                            Action::GetInstance()->Start(12);
                        }
                    } else {
                        KickBall = 0;
                    }
                } else {
                    m_KickBallCount = 0;
                    KickBall = 0;
                    m_GoalFBStep = m_FitFBStep;
                    m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
                }
            } else {
                m_KickBallCount = 0;
                KickBall = 0;
                m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
                if (m_GoalFBStep < m_FollowMinFBStep)
                    m_GoalFBStep = m_FollowMinFBStep;
                m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
            }
        } else {
            m_KickBallCount = 0;
            KickBall = 0;
            m_GoalFBStep = 0;
            m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
        }
    }
    if (abs(m_GoalRLTurn) < m_follow_threshhold) {
        if (Walking::GetInstance()->IsRunning())
            Walking::GetInstance()->Stop();
        else {
            if (m_KickBallCount < m_KickBallMaxCount)
                m_KickBallCount++;
        }

    } else {

        if (!Walking::GetInstance()->IsRunning()) {
            m_FBStep = 0;
            m_RLTurn = 0;
            m_KickBallCount = 0;
            KickBall = 0;
            Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_RLTurn;
            Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
            Walking::GetInstance()->Start();
        } else {
            if (m_FBStep < m_GoalFBStep)
                m_FBStep += m_UnitFBStep;
            else if (m_FBStep > m_GoalFBStep)
                m_FBStep = m_GoalFBStep;

            if (m_RLTurn < m_GoalRLTurn)
                m_RLTurn += m_UnitRLTurn;
            else if (m_RLTurn > m_GoalRLTurn)
                m_RLTurn -= m_UnitRLTurn;
            Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_RLTurn;
            Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
        }
    }
}

void GoalieBehavior::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, GOALIE_SECTION);
}

void GoalieBehavior::LoadINISettings(minIni* ini, const std::string& section) {
    int value = -2;

    if ((value = ini->geti(section, "follow_threshhold", INVALID_VALUE)) != INVALID_VALUE) m_follow_threshhold = value;
    if ((value = ini->geti(section, "edge_dist_threshhold", INVALID_VALUE)) != INVALID_VALUE) m_edge_dist_threshhold = value;
    if ((value = ini->geti(section, "max_speed", INVALID_VALUE)) != INVALID_VALUE) m_FollowMaxRLTurn = value;
    if ((value = ini->geti(section, "right_x", INVALID_VALUE)) != INVALID_VALUE) m_right_x = value;
    if ((value = ini->geti(section, "right_y", INVALID_VALUE)) != INVALID_VALUE) m_right_y = value;
    if ((value = ini->geti(section, "left_x", INVALID_VALUE)) != INVALID_VALUE) m_left_x = value;
    if ((value = ini->geti(section, "left_y", INVALID_VALUE)) != INVALID_VALUE) m_left_y = value;

    m_BallFinder.LoadINISettings(ini);
}

void GoalieBehavior::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, GOALIE_SECTION);
}

void GoalieBehavior::SaveINISettings(minIni* ini, const std::string& section) {
    ini->put(section, "follow_threshhold", m_follow_threshhold);
    ini->put(section, "edge_dist_threshhold", m_edge_dist_threshhold);
    ini->put(section, "max_speed", m_FollowMaxRLTurn);
    ini->put(section, "right_x", m_right_x);
    ini->put(section, "right_y", m_right_y);
    ini->put(section, "left_x", m_left_x);
    ini->put(section, "left_y", m_left_y);

    m_BallFinder.SaveINISettings(ini);
}
