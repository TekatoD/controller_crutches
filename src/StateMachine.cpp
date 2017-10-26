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
#include "motion/modules/Head.h"
#include "motion/modules/Action.h"
#include "motion/modules/Walking.h"
#include "motion/MotionStatus.h"
#include "motion/MotionManager.h"

using namespace Robot;

StateMachine::StateMachine() {
    m_old_btn = 0;
    m_is_started = false;
    m_role = ROLE_IDLE;
}


void StateMachine::Check(CM730* cm730) {
    if (MotionStatus::FALLEN != STANDUP && m_is_started == 1) {
        Walking::GetInstance()->Stop();
        while (Walking::GetInstance()->IsRunning()) usleep(8000);

        Action::GetInstance()->m_Joint.SetEnableBody(true, true);

        if (MotionStatus::FALLEN == FORWARD)
            Action::GetInstance()->Start(10);   // FORWARD GETUP
        else if (MotionStatus::FALLEN == BACKWARD)
            Action::GetInstance()->Start(11);   // BACKWARD GETUP

        while (Action::GetInstance()->IsRunning()) usleep(8000);

        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    }

    if (m_old_btn == MotionStatus::BUTTON)
        return;

    m_old_btn = MotionStatus::BUTTON;

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

void StateMachine::UpdateLeds(CM730* cm730) const {
    switch (m_role) {
        case (ROLE_SOCCER):
            cm730->WriteByte(CM730::P_LED_PANNEL, 0x01, NULL);
            break;
        case (ROLE_PENALTY):
            cm730->WriteByte(CM730::P_LED_PANNEL, 0x02, NULL);
            break;
        case (ROLE_GOALKEEPER):
            cm730->WriteByte(CM730::P_LED_PANNEL, 0x04, NULL);
            break;
        case (ROLE_IDLE):
        default:
            cm730->WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);
            break;
    }
}

void StateMachine::Enable() {
    m_is_started = true;
    MotionManager::GetInstance()->Reinitialize();
    MotionManager::GetInstance()->SetEnable(true);

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);

    Action::GetInstance()->Start(9);
    while (Action::GetInstance()->IsRunning()) usleep(8000);

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

    MotionManager::GetInstance()->ResetGyroCalibration();
    while (true) {
        if (MotionManager::GetInstance()->GetCalibrationStatus() == 1) {
            break;
        } else if (MotionManager::GetInstance()->GetCalibrationStatus() == -1) {
            MotionManager::GetInstance()->ResetGyroCalibration();
        }
        usleep(8000);
    }
}

void StateMachine::Disable() {
    m_is_started = false;
    Walking::GetInstance()->Stop();
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);

    while (!Action::GetInstance()->Start(15)) usleep(8000);
    while (Action::GetInstance()->IsRunning()) usleep(8000);
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
