#pragma once

#include <cstring>
#include "math/Point.h"

namespace Robot {
    class BallTracker {
    public:

        BallTracker();

        ~BallTracker();

        const Point2D& GetBallPosition() const;

        bool IsNoBall() const;

        void Process(Point2D pos);

        int GetNoBallMaxCount() const;

        void SetNoBallMaxCount(int no_ball_max_count);

        bool IsDebugEnabled() const;

        void EnableDebug(bool debug);

    private:

        int m_NoBallCount;
        int m_NoBallMaxCount;
        Point2D BallPosition;
        bool m_debug{false};

    };
}
