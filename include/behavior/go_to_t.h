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
#include <tool/rate_t.h>
#include "math/point_t.h"
#include "motion/pose2d_t.h"

namespace drwn {
    class go_to_t {
    public:
        static go_to_t* get_instance();

        ~go_to_t() = default;

        bool is_done() const;

        void process(pose2d_t pos);

        float get_max_speed() const;

        void set_max_speed(float max_speed);

        float get_fit_speed() const;

        void set_fit_speed(float fit_speed);

        float get_max_turn() const;

        void set_max_turn(float max_turn);

        float get_step_accel() const;

        void set_step_accel(float step_accel);

        float get_turn_accel() const;

        void set_turn_accel(float turn_accel);

        float get_fit_distance() const;

        void set_fit_distance(float fit_distance);

        float get_distance_var() const;

        void set_distance_var(float distance_var);

        float get_angle_var() const;

        void set_angle_var(float angle_var);

        steady_rate_t::duration get_update_rate() const;

        void set_update_rate(const steady_rate_t::duration& duration);

        bool is_debug_enabled() const;

        void enable_debug(bool debug);

    private:
        go_to_t() = default;

    private:
        bool m_debug{false};

        steady_rate_t m_update_rate{std::chrono::milliseconds(500)};

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


