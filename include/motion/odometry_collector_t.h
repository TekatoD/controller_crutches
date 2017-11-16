/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once

#include "pose2d_t.h"

namespace drwn {
    class odometry_collector_t {
    public:

        odometry_collector_t();

        odometry_collector_t(pose2d_t initial);

        odometry_collector_t(float x, float y, float theta);

        void odo_translate(pose2d_t offset);

        pose2d_t get_pose() const;

        void set_pose(pose2d_t offset);

        void set_initial(pose2d_t offset);

        void reset();

    private:
        pose2d_t m_pose;
        pose2d_t m_initial;
    };
}

