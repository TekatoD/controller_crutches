/*
 *   BallTracker.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _BALL_TRACKER_H_
#define _BALL_TRACKER_H_

#include <string.h>

#include "math/Point.h"
#include "minIni.h"

#define TRACKER_SECTION "Ball Tracker"

namespace Robot {
    class BallTracker {
    private:
        int m_NoBallCount;
        int m_NoBallMaxCount = 15;

    public:
        Point2D BallPosition;

        BallTracker();

        ~BallTracker();

        void Process(Point2D pos);

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);
    };
}

#endif
