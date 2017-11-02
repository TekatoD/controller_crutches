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
#include <gamecontroller/game_controller_t.h>
#include <state_machine_t.h>
#include <iostream>
#include <motion/motion_status_t.h>
#include "soccer_behavior_t.h"

using namespace drwn;

soccer_behavior_t::soccer_behavior_t() {
    m_PreviousState = STATE_INITIAL;
}

void soccer_behavior_t::process() {
    // Update CV
    walking_t::GetInstance()->set_move_aim_on(false);
    const pose_2D_t& Spawn = state_machine_t::get_instance()->get_spawn_position();
    const pose_2D_t& Starting = state_machine_t::get_instance()->get_starting_position();
    const pose_2D_t& Odo = walking_t::GetInstance()->get_odo();

//    m_ball_tracker.process(ball);

    const robo_cup_game_control_data_t& State = game_controller_t::get_instance()->game_ctrl_data;

//    if (State.state == STATE_INITIAL || State.state == STATE_FINISHED) {
//        walking_t::get_instance()->set_odo(Spawn);
//        walking_t::get_instance()->stop();
//        return;
//    }
//
//    if (State.state == STATE_READY) {
//        if (m_PreviousState != STATE_INITIAL) {
//            m_PreviousState = STATE_INITIAL;
//            walking_t::get_instance()->set_odo(Spawn);
//        }
//        head_t::get_instance()->joint.set_enable_head_only(true, true);
//        walking_t::get_instance()->joint.set_enable_body_without_head(true, true);
//        pose_2D_t pos = Starting - Odo;
//        m_go_to.process(pos);
//        return;
//    }


    // TODO Player penalised
//    int team = State.teams[0].team_number == game_controller_t::get_instance()->team_number ? 0 : 1;
//    int player = game_controller_t::get_instance()->player_number;
//    if (State.teams[team].players[player].secsTillUnpenalised > 0) {
//        walking_t::get_instance()->stop();
//        return;
//    }

    if (State.state == STATE_SET || State.state == STATE_READY || State.state == STATE_INITIAL) {
        const pose_2D_t& Spawn = state_machine_t::get_instance()->get_spawn_position();
        if (m_ball_tracker.is_no_ball()) {
            head_t::GetInstance()->move_to_home();
        }
        walking_t::GetInstance()->set_odo(Starting);
        walking_t::GetInstance()->stop();
        return;
    }

    if (State.state == STATE_PLAYING) {
//        if (State.kick_off_team != team) {
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
            float free_space = (m_field.get_width() - m_field.get_gate_width()) / 2.0;
            float x_top = m_field.get_width() - free_space;
            float x_bot = x_top - m_field.get_gate_width();

            float pan = motion_status_t::m_current_joints.set_angle(joint_data_t::ID_HEAD_PAN);
            float angle_top = (atan2(m_field.get_length() - Odo.y(), x_top - Odo.x()) - Odo.theta()) / M_PI * 180.0;
            float angle_bot = (atan2(m_field.get_length() - Odo.y(), x_bot - Odo.x()) - Odo.theta()) / M_PI * 180.0;
            angle_bot -= pan;
            angle_top -= pan;
            this->normalize(angle_bot);
            this->normalize(angle_top);

            // Follow the ball
            m_ball_follower.process(m_ball_tracker.get_ball_position(), angle_top, angle_bot);

            if (m_ball_tracker.is_no_ball()) {
                m_ball_searcher.process();
                return;
            } else {
                m_ball_searcher.set_last_position(m_ball_tracker.get_ball_position());
            }

            // Kicking the ball
            if (m_ball_follower.get_kicking_leg() != NO_KICKING) {
                head_t::GetInstance()->joint.set_enable_head_only(true, true);
                action_t::GetInstance()->joint.set_enable_body_without_head(true, true);
                // Kick the ball

                if (m_ball_follower.get_kicking_leg() == RIGHT_LEG_KICK) {
                    action_t::GetInstance()->start(12);   // RIGHT KICK
                } else if (m_ball_follower.get_kicking_leg() == LEFT_LEG_KICK) {
                    action_t::GetInstance()->start(13);   // LEFT KICK
                }
            }
        }
    }
}

void soccer_behavior_t::normalize(float& theta) const {
    while (theta < -180.0) theta += 2.0 * 180.0;
    while (theta > 180.0) theta -= 2.0 * 180.0;
}
