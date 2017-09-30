/**
 * Copyright 2016 Arseniy Ivin <arssivka@yandex.ru>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  @autor arssivka
 *  @date 5/5/17
 */

#include <LinuxCamera.h>
#include <motion/modules/Action.h>
#include <motion/modules/Head.h>
#include <motion/modules/Walking.h>
#include <GameController.h>
#include <StateMachine.h>
#include <iostream>
#include <motion/MotionStatus.h>
#include "SoccerBehavior.h"

using namespace Robot;

SoccerBehavior::SoccerBehavior() {
    m_PreviousState = STATE_INITIAL;
}

void SoccerBehavior::Process() {
    // Update CV
    Walking::GetInstance()->A_MOVE_AIM_ON = false;
    const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
    const Pose2D& Starting = StateMachine::GetInstance()->GetStartingPosition();
    const Pose2D& Odo = Walking::GetInstance()->GetOdo();

    LinuxCamera::GetInstance()->CaptureFrame();
    Image* img = LinuxCamera::GetInstance()->fbuffer->m_HSVFrame;
    Point2D& ball = m_BallFinder.GetPosition(img);
    m_BallTracker.Process(ball);

    const RoboCupGameControlData& State = GameController::GetInstance()->GameCtrlData;

//    if (State.state == STATE_INITIAL || State.state == STATE_FINISHED) {
//        Walking::GetInstance()->SetOdo(Spawn);
//        Walking::GetInstance()->Stop();
//        return;
//    }
//
//    if (State.state == STATE_READY) {
//        if (m_PreviousState != STATE_INITIAL) {
//            m_PreviousState = STATE_INITIAL;
//            Walking::GetInstance()->SetOdo(Spawn);
//        }
//        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
//        Pose2D pos = Starting - Odo;
//        m_GoTo.Process(pos);
//        return;
//    }


    // TODO Player penalised
//    int team = State.teams[0].teamNumber == GameController::GetInstance()->TeamNumber ? 0 : 1;
//    int player = GameController::GetInstance()->PlayerNumber;
//    if (State.teams[team].players[player].secsTillUnpenalised > 0) {
//        Walking::GetInstance()->Stop();
//        return;
//    }

    if (State.state == STATE_SET || State.state == STATE_READY || State.state == STATE_INITIAL) {
        const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
        if (m_BallTracker.IsNoBall()) {
            Head::GetInstance()->MoveToHome();
        }
        Walking::GetInstance()->SetOdo(Starting);
        Walking::GetInstance()->Stop();
        return;
    }

    if (State.state == STATE_PLAYING) {
//        if (State.kickOffTeam != team) {
//            // TODO KickOff
//        }

        if (m_PreviousState != STATE_SET) {
            m_PreviousState = STATE_SET;
            Walking::GetInstance()->SetOdo(Starting);
        }

        if (Action::GetInstance()->IsRunning() == 0) {
            // Switch to head and walking after action
            Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

            // Calculate angles to gate
            float free_space = (m_Field.GetWidth() - m_Field.GetGateWidth()) / 2.0;
            float x_top = m_Field.GetWidth() - free_space;
            float x_bot = x_top - m_Field.GetGateWidth();

            float pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
            float angle_top = (atan2(m_Field.GetLength() - Odo.Y(), x_top - Odo.X()) - Odo.Theta()) / M_PI * 180.0;
            float angle_bot = (atan2(m_Field.GetLength() - Odo.Y(), x_bot - Odo.X()) - Odo.Theta()) / M_PI * 180.0;
            angle_bot -= pan;
            angle_top -= pan;
            this->normalize(angle_bot);
            this->normalize(angle_top);

            // Follow the ball
            m_BallFollower.Process(m_BallTracker.GetBallPosition(), angle_top, angle_bot);

            if (m_BallTracker.IsNoBall()) {
                m_BallSearcher.Process();
                return;
            } else {
                m_BallSearcher.SetLastPosition(m_BallTracker.GetBallPosition());
            }

            // Kicking the ball
            if (m_BallFollower.GetKickingLeg() != NO_KICKING) {
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                // Kick the ball

                if (m_BallFollower.GetKickingLeg() == RIGHT_LEG_KICK) {
                    Action::GetInstance()->Start(12);   // RIGHT KICK
                } else if (m_BallFollower.GetKickingLeg() == LEFT_LEG_KICK) {
                    Action::GetInstance()->Start(13);   // LEFT KICK
                }
            }
        }
    }
}

void SoccerBehavior::normalize(float& m_theta) const {
    while (m_theta < -180.0) m_theta += 2.0 * 180.0;
    while (m_theta > 180.0) m_theta -= 2.0 * 180.0;
}

void SoccerBehavior::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, SOCCER_SECTION);
}

void SoccerBehavior::LoadINISettings(minIni* ini, const std::string& section) {
    m_BallFinder.LoadINISettings(ini);
    m_BallTracker.LoadINISettings(ini);
    m_BallFollower.LoadINISettings(ini);
    m_BallSearcher.LoadINISettings(ini);
    m_GoTo.LoadINISettings(ini);
    m_Field.LoadINISettings(ini);
}


void SoccerBehavior::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, SOCCER_SECTION);
}


void SoccerBehavior::SaveINISettings(minIni* ini, const std::string& section) {
    m_BallFinder.SaveINISettings(ini);
    m_BallTracker.SaveINISettings(ini);
    m_BallFollower.SaveINISettings(ini);
    m_GoTo.SaveINISettings(ini);
    m_Field.SaveINISettings(ini);
}
