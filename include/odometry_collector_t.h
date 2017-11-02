/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once

#include "pose_2D_t.h"

namespace drwn {
    class odometry_collector_t {
    public:

        odometry_collector_t();

        odometry_collector_t(pose_2D_t initial);

        odometry_collector_t(float x, float y, float theta);

        void odo_translate(pose_2D_t offset);

        pose_2D_t get_pose() const;

        void set_pose(pose_2D_t offset);

        void set_initial(pose_2D_t offset);

        void reset();

    private:
        pose_2D_t m_pose;
        pose_2D_t m_initial;
    };
}

