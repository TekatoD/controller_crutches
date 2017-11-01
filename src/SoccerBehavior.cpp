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

#include <motion/modules/action_t.h>
#include <motion/modules/head_t.h>
#include <motion/modules/walking_t.h>
#include <gamecontroller/GameController.h>
#include <StateMachine.h>
#include <iostream>
#include <motion/motion_status_t.h>
#include "SoccerBehavior.h"

using namespace drwn;

SoccerBehavior::SoccerBehavior() {
    m_PreviousState = STATE_INITIAL;
}

void SoccerBehavior::Process() {
    // Update CV
    walking_t::GetInstance()->set_move_aim_on(false);
    const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
    const Pose2D& Starting = StateMachine::GetInstance()->GetStartingPosition();
    const Pose2D& Odo = walking_t::GetInstance()->get_odo();

//    m_BallTracker.Process(ball);

    const RoboCupGameControlData& State = GameController::GetInstance()->GameCtrlData;

//    if (State.state == STATE_INITIAL || State.state == STATE_FINISHED) {
//        walking_t::GetInstance()->set_odo(Spawn);
//        walking_t::GetInstance()->stop();
//        return;
//    }
//
//    if (State.state == STATE_READY) {
//        if (m_PreviousState != STATE_INITIAL) {
//            m_PreviousState = STATE_INITIAL;
//            walking_t::GetInstance()->set_odo(Spawn);
//        }
//        head_t::GetInstance()->joint.set_enable_head_only(true, true);
//        walking_t::GetInstance()->joint.set_enable_body_without_head(true, true);
//        Pose2D pos = Starting - Odo;
//        m_GoTo.Process(pos);
//        return;
//    }


    // TODO Player penalised
//    int team = State.teams[0].teamNumber == GameController::GetInstance()->TeamNumber ? 0 : 1;
//    int player = GameController::GetInstance()->PlayerNumber;
//    if (State.teams[team].players[player].secsTillUnpenalised > 0) {
//        walking_t::GetInstance()->stop();
//        return;
//    }

    if (State.state == STATE_SET || State.state == STATE_READY || State.state == STATE_INITIAL) {
        const Pose2D& Spawn = StateMachine::GetInstance()->GetSpawnPosition();
        if (m_BallTracker.IsNoBall()) {
            head_t::GetInstance()->move_to_home();
        }
        walking_t::GetInstance()->set_odo(Starting);
        walking_t::GetInstance()->stop();
        return;
    }

    if (State.state == STATE_PLAYING) {
//        if (State.kickOffTeam != team) {
//            // TODO KickOff
//        }

        if (m_PreviousState != STATE_SET) {
            m_PreviousState = STATE_SET;
            walking_t::GetInstance()->set_odo(Starting);
        }

        if (action_t::GetInstance()->is_running() == 0) {
            // Switch to head and walking after action
            head_t::GetInstance()->joint.set_enable_head_only(true, true);
            walking_t::GetInstance()->joint.set_enable_body_without_head(true, true);

            // Calculate angles to gate
            float free_space = (m_Field.GetWidth() - m_Field.GetGateWidth()) / 2.0;
            float x_top = m_Field.GetWidth() - free_space;
            float x_bot = x_top - m_Field.GetGateWidth();

            float pan = motion_status_t::m_current_joints.set_angle(joint_data_t::ID_HEAD_PAN);
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
                head_t::GetInstance()->joint.set_enable_head_only(true, true);
                action_t::GetInstance()->joint.set_enable_body_without_head(true, true);
                // Kick the ball

                if (m_BallFollower.GetKickingLeg() == RIGHT_LEG_KICK) {
                    action_t::GetInstance()->start(12);   // RIGHT KICK
                } else if (m_BallFollower.GetKickingLeg() == LEFT_LEG_KICK) {
                    action_t::GetInstance()->start(13);   // LEFT KICK
                }
            }
        }
    }
}

void SoccerBehavior::normalize(float& m_theta) const {
    while (m_theta < -180.0) m_theta += 2.0 * 180.0;
    while (m_theta > 180.0) m_theta -= 2.0 * 180.0;
}
