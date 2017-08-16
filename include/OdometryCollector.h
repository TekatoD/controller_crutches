/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once

#include "Pose2D.h"

namespace Robot {
    class OdometryCollector {
    public:

        OdometryCollector();

        OdometryCollector(Pose2D initial);

        OdometryCollector(float x, float y, float theta);

        void odoTranslate(Pose2D offset);

        Pose2D GetPose() const;

        void SetPose(Pose2D offset);

        void SetInitial(Pose2D offset);

        void Reset();

    private:
        Pose2D m_pose;
        Pose2D m_initial;
    };
}

