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

#pragma once


#include <cmath>
#include "math/Point.h"
#include "Pose2D.h"
#include "minIni.h"

#define GOTO_SECTION ("Goto")

namespace Robot {
    class GoTo {
    private:
        float m_MaxSpeed;
        float m_FitSpeed;
        float m_MaxTurn;
        float m_StepAccel;
        float m_TurnAccel;
        float m_DodeAngle;

        float m_FitDistance;
        float m_DistanceVar;
        float m_AngleVar;

        float m_GoalMaxSpeed;
        float m_GoalTurn;
        float m_X;
        float m_Y;
        float m_A;


        bool m_Done;
    public:

        GoTo();

        ~GoTo() {}

        bool IsDone() const;

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

        void Process(Pose2D pos);
    };
}


