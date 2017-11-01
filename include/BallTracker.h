#pragma once

#include <cstring>
#include "math/point_t.h"

namespace drwn {
    class BallTracker {
    public:

        BallTracker();

        ~BallTracker();

        const point_2D_t& GetBallPosition() const;

        bool IsNoBall() const;

        void Process(point_2D_t pos);

        int GetNoBallMaxCount() const;

        void SetNoBallMaxCount(int no_ball_max_count);

        bool IsDebugEnabled() const;

        void EnableDebug(bool debug);

    private:

        int m_NoBallCount;
        int m_NoBallMaxCount;
        point_2D_t BallPosition;
        bool m_debug{false};

    };
}
