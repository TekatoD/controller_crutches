#pragma once

#include <cstring>

#include "math/Point.h"

//#define TRACKER_SECTION "Ball Tracker"

namespace Robot {
    class BallTracker {
    private:
        int m_NoBallCount;
        int m_NoBallMaxCount;

        Point2D BallPosition;

    public:

        BallTracker();

        ~BallTracker();

        const Point2D& GetBallPosition() const;

        bool IsNoBall() const;

        void Process(Point2D pos);

    };
}
