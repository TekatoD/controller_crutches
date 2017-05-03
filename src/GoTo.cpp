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
 *  @date 5/3/17
 */

#include "GoTo.h"
#include "Walking.h"

#define PI (3.14159265)


bool Robot::GoTo::IsDone() const {
    return m_Done;
}

void Robot::GoTo::Process(Robot::Pose2D pos) {
    m_Done = true;
    double dist = hypot(pos.getX(), pos.getY());
    double angle = atan2(pos.getY(), pos.getX()) / PI * 180.0;

    if (!Walking::GetInstance()->IsRunning() ||
        Walking::GetInstance()->X_MOVE_AMPLITUDE != m_X ||
        Walking::GetInstance()->Y_MOVE_AMPLITUDE != m_Y ||
        Walking::GetInstance()->A_MOVE_AMPLITUDE != m_A) {
        m_X = Walking::GetInstance()->X_MOVE_AMPLITUDE;
        m_Y = Walking::GetInstance()->Y_MOVE_AMPLITUDE;
        m_A = Walking::GetInstance()->A_MOVE_AMPLITUDE;
    }

    if (dist > m_DistanceVar) {
        m_GoalMaxSpeed = (dist < m_FitDistance) ? m_FitMaxSpeed : m_MaxSpeed;

        double x_speed = pos.getX() / dist * m_GoalMaxSpeed;
        double y_speed = pos.getY() / dist * m_GoalMaxSpeed;

        m_X += (m_X - x_speed > 0 ? -m_UnitFBStep : m_UnitFBStep);
        m_Y += (m_Y - y_speed > 0 ? -m_UnitFBStep : m_UnitFBStep);

        m_Done = false;
    } else {
        m_X = 0;
        m_Y = 0;
    }

    if (angle > 0 && angle > m_AngleVar ||
            angle < 0 && angle < -m_AngleVar) {
        m_GoalRLTurn = m_MaxTurn;
        double a_speed = (angle > 0) ? m_GoalRLTurn : -m_GoalRLTurn;
        m_A += ((m_A - a_speed) > 0) ? -m_UnitRLTurn : m_UnitRLTurn;
        m_Done = false;
    } else {
        m_A = 0;
    }


    if (!m_Done) {
        Walking::GetInstance()->X_MOVE_AMPLITUDE = m_X;
        Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_Y;
        Walking::GetInstance()->Start();
    } else {
        Walking::GetInstance()->Stop();
    }
}

Robot::GoTo::GoTo() {
    m_MaxSpeed = 30.0;
    m_MaxTurn = 35.0;
    m_UnitFBStep = 0.3;
    m_UnitRLTurn = 1.0;

    m_FitDistance = 200.0;
    m_DistanceVar = 50.0;
    m_AngleVar = 10.0;

    m_GoalMaxSpeed = 0.0;
    m_GoalRLTurn = 0.0;

    m_X = 0.0;
    m_Y = 0.0;
    m_A = 0.0;

    m_Done = false;
}
