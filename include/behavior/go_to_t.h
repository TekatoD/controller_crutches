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
#include "motion/pose2d_t.h"

namespace drwn {
    class go_to_t {
    public:
        static go_to_t* get_instance();

        ~go_to_t() = default;

        bool is_done() const;

        void process(pose2d_t pos);

    private:
        go_to_t() = default;

    private:
        float m_max_speed{7.0f};
        float m_fit_speed{3.0f};
        float m_max_turn{7.0f};
        float m_step_accel{0.3f};
        float m_turn_accel{0.3f};

        float m_fit_distance{200.0f};
        float m_distance_var{50.0f};
        float m_angle_var{10.0f};

        float m_goal_max_speed{0.0f};
        float m_goal_turn{0.0f};

        float m_x{0.0f};
        float m_y{0.0f};
        float m_a{0.0f};

        bool m_done{false};
    };
}


