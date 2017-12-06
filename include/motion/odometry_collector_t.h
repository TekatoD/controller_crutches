/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once

#include "pose2d_t.h"

namespace drwn {
    class odometry_collector_t {
    public:
        odometry_collector_t() = default;

        explicit odometry_collector_t(const pose2d_t& initial);

        odometry_collector_t(float x, float y, float theta);

        void odo_translate(const pose2d_t& offset);

        const pose2d_t& get_pose() const;

        void set_pose(const pose2d_t& offset);

        bool is_debug_enabled() const;

        void enable_debug(bool debug);

    private:
        bool m_debug{false};
        pose2d_t m_pose{0, 0, 0};
    };
}

