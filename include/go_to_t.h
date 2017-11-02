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
#include "math/point_t.h"
#include "pose_2D_t.h"

#define GOTO_SECTION ("Goto")

namespace drwn {
    class go_to_t {
    private:
        float m_max_speed;
        float m_fit_speed;
        float m_max_turn;
        float m_step_accel;
        float m_turn_accel;
        float m_dode_angle;

        float m_fit_distance;
        float m_distance_var;
        float m_angle_var;

        float m_goal_max_speed;
        float m_goal_turn;
        float m_x;
        float m_y;
        float m_a;


        bool m_done;
    public:

        go_to_t();

        ~go_to_t() {}

        bool is_done() const;

        void process(pose_2D_t pos);
    };
}


