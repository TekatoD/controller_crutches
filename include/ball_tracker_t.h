#pragma once

#include <cstring>
#include "math/point_t.h"

namespace drwn {
    class ball_tracker_t {
    public:

        ball_tracker_t();

        ~ball_tracker_t();

        const point_2D_t& get_ball_position() const;

        bool is_no_ball() const;

        void process(point_2D_t pos);

        int get_no_ball_max_count() const;

        void set_no_ball_max_count(int no_ball_max_count);

        bool is_debug_enabled() const;

        void enable_debug(bool debug);

    private:

        int m_no_ball_count;
        int m_no_ball_max_count;
        point_2D_t m_ball_position;
        bool m_debug{false};

    };
}
