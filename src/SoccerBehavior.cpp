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
#include "SoccerBehavior.h"

using namespace Robot;

SoccerBehavior::SoccerBehavior(CM730& cm730)
        : m_CM730(cm730) {}

void SoccerBehavior::Process() {
    // Update CV
    LinuxCamera::GetInstance()->CaptureFrame();
    m_BallTracker.Process(m_BallFinder.GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

    const RoboCupGameControlData& State = GameController::GetInstance()->GameCtrlData;

    if (State.state == STATE_INITIAL || State.state == STATE_FINISHED) {
        Walking::GetInstance()->Stop();
        return;
    }

    if (State.state == STATE_READY) {
        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

        const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
        const Pose2D& Starting = StateMachine::GetInstance()->GetStartingPosition();
        const Pose2D& Odo = Walking::GetInstance()->GetOdo();

        auto pos = Starting - Spawn - Odo;
        m_GoTo.Process(pos);
        return;
    }

    if (State.state == STATE_SET) {
        const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
        Walking::GetInstance()->SetOdo(Spawn);
        return;
    }

    if (State.state == STATE_PLAYING) {
        if (Action::GetInstance()->IsRunning() == 0) {
            // Switch to head and walking after action
            Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
            // Follow the ball
            m_BallFollower.Process(m_BallTracker.ball_position);

            if (m_BallFollower.KickBall != 0) {
                std::cout << "Balls!" << std::endl;
            }
            // Kicking the ball
            if (m_BallFollower.KickBall != 0) {
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                // Kick the ball
                if (m_BallFollower.KickBall == -1) {
                    Action::GetInstance()->Start(12);   // RIGHT KICK
                } else if (m_BallFollower.KickBall == 1) {
                    Action::GetInstance()->Start(13);   // LEFT KICK
                }
            }
        }
    }
}

void SoccerBehavior::LoadINISettings(minIni *ini) {
    LoadINISettings(ini, SOCCER_SECTION);
}

void SoccerBehavior::LoadINISettings(minIni *ini, const std::string& section) {
    m_BallFinder.LoadINISettings(ini);
    m_GoTo.LoadINISettings(ini);
    m_Field.LoadINISettings(ini);
    m_BallFollower.LoadINISettings(ini);
}


void SoccerBehavior::SaveINISettings(minIni *ini) {
    SaveINISettings(ini, SOCCER_SECTION);
}


void SoccerBehavior::SaveINISettings(minIni *ini, const std::string& section) {
    m_BallFinder.SaveINISettings(ini);
    m_GoTo.SaveINISettings(ini);
    m_Field.SaveINISettings(ini);
    m_BallFollower.LoadINISettings(ini);
}
