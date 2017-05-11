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
        : m_CM730(cm730) {
    m_AimAAmplitude = 20;
    m_AimRLAmplitude = 20;
}

void SoccerBehavior::Process() {
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
        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
        auto pos = Starting - Odo;
        m_GoTo.Process(pos);
        return;
    }

    if (State.state == STATE_SET) {
        const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
        Walking::GetInstance()->SetOdo(Starting);
        Walking::GetInstance()->Stop();
        return;
    }

    if (State.state == STATE_PLAYING) {
        if (Action::GetInstance()->IsRunning() == 0) {
            // Switch to head and walking after action
            Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
            // Follow the ball
            m_BallFollower.Process(m_BallTracker.BallPosition);

            // Kicking the ball
            if (m_BallFollower.KickBall != 0) {
                auto free_space = (m_Field.GetWidth() - m_Field.GetGateWidth()) / 2.0;
                double y_top = m_Field.GetWidth() - free_space;
                double y_bot = y_top - m_Field.GetGateWidth();

                double angle_top = atan2(y_top - Odo.Y(), Odo.X() - m_Field.GetLength()) - Odo.Theta();
                double angle_bot = atan2(y_bot - Odo.Y(), Odo.X() - m_Field.GetLength()) - Odo.Theta();

                if (angle_top > 0) {
                    Walking::GetInstance()->A_MOVE_AIM_ON = true;
                    Walking::GetInstance()->A_MOVE_AMPLITUDE = m_AimAAmplitude;
                    Walking::GetInstance()->Y_MOVE_AMPLITUDE = -m_AimRLAmplitude;
                    Walking::GetInstance()->Start();
                    return;
                } else if (angle_bot < 0) {
                    Walking::GetInstance()->A_MOVE_AIM_ON = true;
                    Walking::GetInstance()->A_MOVE_AMPLITUDE = -m_AimAAmplitude;
                    Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_AimRLAmplitude;
                    Walking::GetInstance()->Start();
                    return;
                }
                Walking::GetInstance()->Stop();

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
    double dvalue;
    if ((dvalue = ini->getd(section, "aim_rl_amplitude", INVALID_VALUE)) != INVALID_VALUE) m_AimRLAmplitude = dvalue;
    if ((dvalue = ini->getd(section, "aim_a_amplitude", INVALID_VALUE)) != INVALID_VALUE) m_AimAAmplitude = dvalue;


    m_BallFinder.LoadINISettings(ini);
    m_BallTracker.LoadINISettings(ini);
    m_GoTo.LoadINISettings(ini);
    m_Field.LoadINISettings(ini);
    m_BallFollower.LoadINISettings(ini);
}


void SoccerBehavior::SaveINISettings(minIni *ini) {
    SaveINISettings(ini, SOCCER_SECTION);
}


void SoccerBehavior::SaveINISettings(minIni *ini, const std::string& section) {
    ini->put(section, "aim_rl_amplitude", m_AimRLAmplitude);
    ini->put(section, "aim_a_amplitude", m_AimAAmplitude);

    m_BallFinder.SaveINISettings(ini);
    m_BallTracker.SaveINISettings(ini);
    m_GoTo.SaveINISettings(ini);
    m_Field.SaveINISettings(ini);
    m_BallFollower.LoadINISettings(ini);


}
