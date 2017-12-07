#pragma once

#include <cstring>
#include <tool/rate_t.h>
#include "math/point_t.h"

namespace drwn {
    class ball_tracker_t {
    public:
        static ball_tracker_t* get_instance();

        const point2d_t& get_ball_position() const noexcept;

        bool is_no_ball() const noexcept;

        void process(point2d_t pos);

        std::chrono::duration<int64_t, std::nano> get_no_ball_duration() const noexcept;

        void set_no_ball_duration(steady_rate_t::duration duration);

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

        float get_distance_threshold() const;

        void set_distance_threshold(float distance_threshold);

    private:
        ball_tracker_t() = default;

    private:
        steady_rate_t m_no_ball_rate{std::chrono::seconds(1)};
        point2d_t m_ball_position{-1, -1};
        float m_distance_threshold{80.0f};
        bool m_debug{false};
        bool m_no_ball{true};

    };
}
