/**
 *  @autor tekatod
 *  @date 5/5/17
 */

#include <iostream>
#include <motion/modules/Action.h>
#include <LinuxCamera.h>
#include <GameController.h>
#include <StateMachine.h>
#include "motion/modules/Head.h"
#include "motion/modules/Walking.h"
#include "GoalieBehavior.h"
#include "motion/MotionStatus.h"


using namespace Robot;


GoalieBehavior::GoalieBehavior() {
    m_KickBallMaxCount = 10;
    m_KickBallCount = 0;

    m_KickTopAngle = -5.0;
    m_KickRightAngle = -30.0;
    m_KickLeftAngle = 30.0;

    m_FollowMaxRL = 30.0;
    m_FitMaxRLTurn = 30.0;
    m_UnitRLTurn = 1.0;

    m_GoalRLTurn = 0;
    m_RLTurn = 0;
    m_EdgeDistThreshhold = 80.0;
    m_FollowThreshhold = 2;
    m_PreviousState = STATE_INITIAL;
    m_BallSearcher.DisableWalking();
}


GoalieBehavior::~GoalieBehavior() {
}


void GoalieBehavior::Process() {
    // Update CV
    Walking::GetInstance()->A_MOVE_AIM_ON = false;
    const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
    const Pose2D& Starting = StateMachine::GetInstance()->GetStartingPosition();
    const Pose2D& Odo = Walking::GetInstance()->GetOdo();

    LinuxCamera::GetInstance()->CaptureFrame();
    m_BallTracker.Process(m_BallFinder.GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

    const RoboCupGameControlData& State = GameController::GetInstance()->GameCtrlData;

    if (State.state == STATE_INITIAL || State.state == STATE_FINISHED) {
        Walking::GetInstance()->SetOdo(Spawn);
        Walking::GetInstance()->Stop();
        return;
    }

    if (State.state == STATE_READY) {
        if (m_PreviousState != STATE_INITIAL) {
            m_PreviousState = STATE_INITIAL;
            Walking::GetInstance()->SetOdo(Spawn);
        }
        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
        Pose2D pos = Starting - Odo;
        m_GoTo.Process(pos);
        return;
    }

    if (State.state == STATE_SET) {
        const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
        if (m_BallTracker.IsNoBall()) {
            Head::GetInstance()->MoveToHome();
        }
        Walking::GetInstance()->SetOdo(Starting);
        Walking::GetInstance()->Stop();
        return;
    }

    if (State.state == STATE_PLAYING) {
        if (m_PreviousState != STATE_SET) {
            m_PreviousState = STATE_SET;
            Walking::GetInstance()->SetOdo(Starting);
        }
        double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        double free_space = (m_Field.GetWidth() - m_Field.GetGateWidth()) / 2.0;
        double x_top = m_Field.GetWidth() - free_space;
        double x_bot = x_top - m_Field.GetGateWidth();
        bool near_edge = false;
        if(pan < 0) {
            if (Walking::GetInstance()->GetOdo().X() >= x_top - m_EdgeDistThreshhold) {
                Walking::GetInstance()->Stop();
                near_edge = true;
            }
        } else {
            if (Walking::GetInstance()->GetOdo().X() <= x_bot + m_EdgeDistThreshhold) {
                Walking::GetInstance()->Stop();
                near_edge = true;
            }
        }
        if (Action::GetInstance()->IsRunning() == 0 && !near_edge) {
            Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
            LinuxCamera::GetInstance()->CaptureFrame();
            m_BallTracker.Process(m_BallFinder.GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

            if (m_BallTracker.IsNoBall()) {
                m_BallSearcher.Process();
                return;
            } else {
                m_BallSearcher.SetLastPosition(m_BallTracker.GetBallPosition());

                double pan_range = Head::GetInstance()->GetLeftLimitAngle();
                double pan_percent = pan / pan_range;

                double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
                double tilt_min = Head::GetInstance()->GetBottomLimitAngle();

                if (pan > m_KickRightAngle && pan < m_KickLeftAngle) {
                    if (tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE)) {
                        if (m_BallTracker.GetBallPosition().Y < m_KickTopAngle) {
                            m_GoalRLTurn = 0;

                            if (m_KickBallCount >= m_KickBallMaxCount) {
                                m_RLTurn = 0;
                                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                                Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                                if (pan > 0) {// left
                                    Action::GetInstance()->Start(13);
                                } else {// Right
                                    Action::GetInstance()->Start(12);
                                }
                            }
                        } else {
                            m_KickBallCount = 0;
                            m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
                        }
                    } else {
                        m_KickBallCount = 0;
                        m_GoalRLTurn = m_FollowMaxRL * pan_percent;
                    }
                } else {
                    m_KickBallCount = 0;
                    m_GoalRLTurn = m_FollowMaxRL * pan_percent;
                }
            }
            if (abs(m_GoalRLTurn) < m_FollowThreshhold) {
                if (Walking::GetInstance()->IsRunning())
                    Walking::GetInstance()->Stop();
                else {
                    if (m_KickBallCount < m_KickBallMaxCount)
                        m_KickBallCount++;
                }

            } else {

                if (!Walking::GetInstance()->IsRunning()) {
                    m_RLTurn = 0;
                    m_KickBallCount = 0;
                    Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_RLTurn;
                    Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
                    Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
                    Walking::GetInstance()->Start();
                } else {
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
    }
}

void GoalieBehavior::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, GOALIE_SECTION);
}

void GoalieBehavior::LoadINISettings(minIni* ini, const std::string& section) {
    double value = -2;

    if ((value = ini->getd(section, "follow_threshhold", INVALID_VALUE)) != INVALID_VALUE) m_FollowThreshhold = value;
    if ((value = ini->getd(section, "edge_dist_threshhold", INVALID_VALUE)) != INVALID_VALUE) m_EdgeDistThreshhold = value;
    if ((value = ini->getd(section, "max_speed", INVALID_VALUE)) != INVALID_VALUE) m_FollowMaxRL = value;

    m_BallFinder.LoadINISettings(ini);
    m_BallTracker.LoadINISettings(ini);
    m_BallSearcher.LoadINISettings(ini);
    m_Field.LoadINISettings(ini);
}

void GoalieBehavior::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, GOALIE_SECTION);
}

void GoalieBehavior::SaveINISettings(minIni* ini, const std::string& section) {
    ini->put(section, "follow_threshhold", m_FollowThreshhold);
    ini->put(section, "edge_dist_threshhold", m_EdgeDistThreshhold);
    ini->put(section, "max_speed", m_FollowMaxRL);

    m_BallFinder.SaveINISettings(ini);
    m_BallTracker.SaveINISettings(ini);
    m_BallSearcher.SaveINISettings(ini);
    m_Field.SaveINISettings(ini);
}
