/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>

#include "../include/StatusCheck.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "MotionStatus.h"
#include "MotionManager.h"

using namespace Robot;

int StatusCheck::m_cur_mode = READY;
int StatusCheck::m_old_btn = 0;
int StatusCheck::m_is_started = 0;


void StatusCheck::Check(CM730& cm730) {
    if (MotionStatus::FALLEN != STANDUP && m_cur_mode == SOCCER && m_is_started == 1) {
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

    if (m_old_btn & BTN_MODE) {
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
