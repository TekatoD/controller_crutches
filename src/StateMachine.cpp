/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>
#include <gamecontroller/GameController.h>

#include "StateMachine.h"
#include "motion/modules/head_t.h"
#include "motion/modules/action_t.h"
#include "motion/modules/walking_t.h"
#include "motion/motion_status_t.h"
#include "motion/motion_manager_t.h"

using namespace drwn;

StateMachine::StateMachine() {
    m_old_btn = 0;
    m_is_started = false;
    m_role = ROLE_IDLE;
}


void StateMachine::Check(CM730_t* cm730) {
    if (motion_status_t::FALLEN != STANDUP && m_is_started == 1) {
        walking_t::GetInstance()->stop();
        while (walking_t::GetInstance()->is_running()) usleep(8000);

        action_t::GetInstance()->joint.set_enable_body(true, true);

        if (motion_status_t::FALLEN == FORWARD)
            action_t::GetInstance()->start(10);   // FORWARD GETUP
        else if (motion_status_t::FALLEN == BACKWARD)
            action_t::GetInstance()->start(11);   // BACKWARD GETUP

        while (action_t::GetInstance()->is_running()) usleep(8000);

        head_t::GetInstance()->joint.set_enable_head_only(true, true);
        walking_t::GetInstance()->joint.set_enable_body_without_head(true, true);
    }

    if (m_old_btn == motion_status_t::BUTTON)
        return;

    m_old_btn = motion_status_t::BUTTON;

    if (m_old_btn & BTN_START) {
        if (m_is_started) {
            Disable();
        } else {
            Enable();
        }
    }

    if (m_old_btn & BTN_STATE) {
        if (!m_is_started) {
            bool penalize_switched = false;
            switch (m_role) {
                case ROLE_IDLE:
                    m_role = ROLE_SOCCER;
                    penalize_switched = false;
                    break;
                case ROLE_SOCCER:
                    m_role = ROLE_PENALTY;
                    penalize_switched = true;
                    break;
                case ROLE_PENALTY:
                    m_role = ROLE_GOALKEEPER;
                    penalize_switched = true;
                    break;
                case ROLE_GOALKEEPER:
                default:
                    m_role = ROLE_IDLE;
                    break;
            }

            usleep(10000);

            if (penalize_switched) {
                if (m_role == ROLE_PENALTY) {
                    GameController::GetInstance()->SendPenalise();
                } else {
                    GameController::GetInstance()->SendUnpenalise();
                }
            }

            UpdateLeds(cm730);
        } else {
            Disable();
        }
    }
}

void StateMachine::UpdateLeds(CM730_t* cm730) const {
    switch (m_role) {
        case (ROLE_SOCCER):
            cm730->write_byte(CM730_t::P_LED_PANNEL, 0x01, NULL);
            break;
        case (ROLE_PENALTY):
            cm730->write_byte(CM730_t::P_LED_PANNEL, 0x02, NULL);
            break;
        case (ROLE_GOALKEEPER):
            cm730->write_byte(CM730_t::P_LED_PANNEL, 0x04, NULL);
            break;
        case (ROLE_IDLE):
        default:
            cm730->write_byte(CM730_t::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);
            break;
    }
}

void StateMachine::Enable() {
    m_is_started = true;
    motion_manager_t::GetInstance()->reinitialize();
    motion_manager_t::GetInstance()->set_enable(true);

    action_t::GetInstance()->joint.set_enable_body(true, true);

    action_t::GetInstance()->start(9);
    while (action_t::GetInstance()->is_running()) usleep(8000);

    head_t::GetInstance()->joint.set_enable_head_only(true, true);
    walking_t::GetInstance()->joint.set_enable_body_without_head(true, true);

    motion_manager_t::GetInstance()->reset_gyro_calibration();
    while (true) {
        if (motion_manager_t::GetInstance()->get_calibration_status() == 1) {
            break;
        } else if (motion_manager_t::GetInstance()->get_calibration_status() == -1) {
            motion_manager_t::GetInstance()->reset_gyro_calibration();
        }
        usleep(8000);
    }
}

void StateMachine::Disable() {
    m_is_started = false;
    walking_t::GetInstance()->stop();
    action_t::GetInstance()->joint.set_enable_body(true, true);

    while (!action_t::GetInstance()->start(15)) usleep(8000);
    while (action_t::GetInstance()->is_running()) usleep(8000);
}

int StateMachine::IsStarted() {
    return m_is_started;
}

const Pose2D& StateMachine::GetSpawnPosition() const {
    return m_spawn_pos;
}

const Pose2D& StateMachine::GetStartingPosition() const {
    return m_starting_pos;
}

Role StateMachine::GetRole() const {
    return m_role;
}

void StateMachine::SetRole(Role role) {
    m_role = role;
}

void StateMachine::SetSpawnPosition(const Pose2D& pos) {
    m_spawn_pos = pos;
}

void StateMachine::SetStartingPosition(const Pose2D& pos) {
    m_starting_pos = pos;
}
