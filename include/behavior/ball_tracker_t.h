#pragma once

#include <cstring>
#include "math/point_t.h"

namespace drwn {
    class ball_tracker_t {
    public:
        static ball_tracker_t* get_instance();

        ~ball_tracker_t();

        const point2d_t& get_ball_position() const noexcept;

        bool is_no_ball() const noexcept;

        void process(point2d_t pos);

        int get_no_ball_max_count() const noexcept;

        void set_no_ball_max_count(int no_ball_max_count);

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        ball_tracker_t();

    private:
        int m_no_ball_count;
        int m_no_ball_max_count;
        point2d_t m_ball_position;
        bool m_debug{false};

    };
}
