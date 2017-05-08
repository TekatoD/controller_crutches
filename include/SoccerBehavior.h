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

#pragma once


#include "CM730.h"
#include "ColorFinder.h"
#include "BallTracker.h"
#include "BallFollower.h"
#include "GoTo.h"
#include "Field.h"


#define SOCCER_SECTION "Soccer"

namespace Robot {
    class SoccerBehavior {
    public:
        SoccerBehavior(CM730& cm730);

        void Process();

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

    protected:
        CM730& m_CM730;
        ColorFinder m_BallFinder;
        BallTracker m_BallTracker;
        BallFollower m_BallFollower;
        GoTo m_GoTo;

        Field m_Field;
    };
}


