/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>
#include <GameController.h>

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


void StateMachine::Check(CM730& cm730) {
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

void StateMachine::UpdateLeds(CM730& cm730) const {
    switch (m_role) {
        case (ROLE_SOCCER):
            cm730.WriteByte(CM730::P_LED_PANNEL, 0x01, NULL);
            break;
        case (ROLE_PENALTY):
            cm730.WriteByte(CM730::P_LED_PANNEL, 0x02, NULL);
            break;
        case (ROLE_GOALKEEPER):
            cm730.WriteByte(CM730::P_LED_PANNEL, 0x04, NULL);
            break;
        case (ROLE_IDLE):
        default:
            cm730.WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);
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


void StateMachine::LoadINISettings(minIni *ini) {
    LoadINISettings(ini, CNTRL_SECTION);
}


void StateMachine::LoadINISettings(minIni *ini, const std::string& section) {
    int value = -2;
    if ((value = ini->geti(section, "role", INVALID_VALUE)) != INVALID_VALUE) {
        switch (value) {
            case (ROLE_SOCCER):
                m_role = ROLE_SOCCER;
                break;
            case (ROLE_PENALTY):
                m_role = ROLE_PENALTY;
                break;
            case (ROLE_GOALKEEPER):
                m_role = ROLE_GOALKEEPER;
                break;
            default:
                m_role = ROLE_IDLE;
        }
    }
    if ((value = ini->geti(section, "spawn_x", INVALID_VALUE)) != INVALID_VALUE) m_spawn_pos.setX(value);
    if ((value = ini->geti(section, "spawn_y", INVALID_VALUE)) != INVALID_VALUE) m_spawn_pos.setY(value);
    if ((value = ini->geti(section, "spawn_theta", INVALID_VALUE)) != INVALID_VALUE)
        m_spawn_pos.setTheta(value / 180.0 * M_PI);

    if ((value = ini->geti(section, "starting_x", INVALID_VALUE)) != INVALID_VALUE) m_starting_pos.setX(value);
    if ((value = ini->geti(section, "starting_y", INVALID_VALUE)) != INVALID_VALUE) m_starting_pos.setY(value);
    if ((value = ini->geti(section, "starting_theta", INVALID_VALUE)) != INVALID_VALUE)
        m_starting_pos.setTheta(value / 180.0 * M_PI);
}


void StateMachine::SaveINISettings(minIni *ini) {
    SaveINISettings(ini, CNTRL_SECTION);
}


void StateMachine::SaveINISettings(minIni *ini, const std::string& section) {
    ini->put(section, "role", m_role);
    ini->put(section, "spawn_x", m_spawn_pos.X());
    ini->put(section, "spawn_y", m_spawn_pos.Y());
    ini->put(section, "spawn_theta", m_spawn_pos.Theta() / M_PI * 180.0);
    ini->put(section, "starting_x", m_starting_pos.X());
    ini->put(section, "starting_y", m_starting_pos.Y());
    ini->put(section, "starting_theta", m_starting_pos.Theta() / M_PI * 180.0);
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
