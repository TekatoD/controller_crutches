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


#include "hw/CM730_t.h"
#include "BallTracker.h"
#include "BallFollower.h"
#include "GoTo.h"
#include "Field.h"
#include "BallSearcher.h"


#define SOCCER_SECTION "Soccer"

namespace drwn {
    class SoccerBehavior {
    public:
        SoccerBehavior();

        void Process();

    protected:
        BallTracker m_BallTracker;
        BallFollower m_BallFollower;
        BallSearcher m_BallSearcher;
        GoTo m_GoTo;
        Field m_Field;
        int m_PreviousState;

        void normalize(float& m_theta) const;
    };
}


