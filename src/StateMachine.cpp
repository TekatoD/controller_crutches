/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>

#include "StateMachine.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "MotionStatus.h"
#include "MotionManager.h"

using namespace Robot;

int StateMachine::m_old_btn = 0;
int StateMachine::m_is_started = 0;
int StateMachine::m_role = ROLE_UNKNOWN;
Pose2D StateMachine::m_spawn_pos;
Pose2D StateMachine::m_starting_pos;


void StateMachine::Check(CM730& cm730) {
    if (MotionStatus::FALLEN != STANDUP && m_is_started == 1) {
        Walking::GetInstance()->Stop();
        while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);

        Action::GetInstance()->m_Joint.SetEnableBody(true, true);

        if (MotionStatus::FALLEN == FORWARD)
            Action::GetInstance()->Start(10);   // FORWARD GETUP
        else if (MotionStatus::FALLEN == BACKWARD)
            Action::GetInstance()->Start(11);   // BACKWARD GETUP

        while (Action::GetInstance()->IsRunning() == 1) usleep(8000);

        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    }

    if (m_old_btn == MotionStatus::BUTTON)
        return;

    m_old_btn = MotionStatus::BUTTON;

    if (m_old_btn & BTN_STOP) {
        if (m_is_started == 1) {
            m_is_started = 0;
            Walking::GetInstance()->Stop();
            Action::GetInstance()->m_Joint.SetEnableBody(true, true);

            while (Action::GetInstance()->Start(15) == false) usleep(8000);
            while (Action::GetInstance()->IsRunning() == true) usleep(8000);
        }
    }

    if (m_old_btn & BTN_START) {
        if (m_is_started == 0) {
            fprintf(stderr, "Start button pressed.. & started is false.. \n");

            MotionManager::GetInstance()->Reinitialize();
            MotionManager::GetInstance()->SetEnable(true);
            m_is_started = 1;

            Action::GetInstance()->m_Joint.SetEnableBody(true, true);

            Action::GetInstance()->Start(9);
            while (Action::GetInstance()->IsRunning() == true) usleep(8000);

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
        } else {
            fprintf(stderr, "Start button pressed.. & started is true.. \n");
        }
    }
}


void StateMachine::LoadINISettings(minIni *ini) {
    LoadINISettings(ini, CNTRL_SECTION);
}


void StateMachine::LoadINISettings(minIni *ini, const std::string &section) {
    int value = -2;
    if ((value = ini->geti(section, "role", INVALID_VALUE)) != INVALID_VALUE) {
        switch (value) {
            case (ROLE_SOCCER):
            case (ROLE_GOALKEEPER):
                m_role = value;
                break;
            default:
                m_role = ROLE_UNKNOWN;
        }
    }
    if ((value = ini->geti(section, "spawn_x", INVALID_VALUE)) != INVALID_VALUE) m_spawn_pos.setX(value);
    if ((value = ini->geti(section, "spawn_y", INVALID_VALUE)) != INVALID_VALUE) m_spawn_pos.setY(value);
    if ((value = ini->geti(section, "spawn_theta", INVALID_VALUE)) != INVALID_VALUE) m_spawn_pos.setTheta(value / 180 * M_PI);

    if ((value = ini->geti(section, "starting_x", INVALID_VALUE)) != INVALID_VALUE) m_starting_pos.setX(value);
    if ((value = ini->geti(section, "starting_y", INVALID_VALUE)) != INVALID_VALUE) m_starting_pos.setY(value);
    if ((value = ini->geti(section, "starting_theta", INVALID_VALUE)) != INVALID_VALUE) m_starting_pos.setTheta(value / 180 * M_PI);
}


void StateMachine::SaveINISettings(minIni *ini) {
    SaveINISettings(ini, CNTRL_SECTION);
}


void StateMachine::SaveINISettings(minIni *ini, const std::string &section) {
    ini->put(section, "role", m_role);
    ini->put(section, "spawn_x", m_spawn_pos.X());
    ini->put(section, "spawn_y", m_spawn_pos.Y());
    ini->put(section, "spawn_theta", m_spawn_pos.Theta());
    ini->put(section, "starting_x", m_starting_pos.X());
    ini->put(section, "starting_y", m_starting_pos.Y());
    ini->put(section, "starting_theta", m_starting_pos.Theta());
}

int StateMachine::IsStarted() {
    return m_is_started;
}

const Pose2D& StateMachine::SpawnPosition() {
    return m_spawn_pos;
}

const Pose2D& StateMachine::StartingPosition() {
    return m_starting_pos;
}
