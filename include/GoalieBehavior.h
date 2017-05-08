/**
 *  @autor tekatod
 *  @date 5/5/17
 */
#pragma once

#include "math/Point.h"
#include "BallTracker.h"
#include "ColorFinder.h"

#define GOALIE_SECTION ("Goalie")

namespace Robot {
    class GoalieBehavior {
    private:
        int m_NoBallMaxCount;
        int m_NoBallCount;
        int m_KickBallMaxCount;
        int m_KickBallCount;

        double m_MaxFBStep;
        double m_MaxRLStep;
        double m_MaxDirAngle;

        double m_KickTopAngle;
        double m_KickRightAngle;
        double m_KickLeftAngle;

        double m_FollowMaxFBStep;
        double m_FollowMinFBStep;
        double m_FollowMaxRLTurn;
        double m_FitFBStep;
        double m_FitMaxRLTurn;
        double m_UnitFBStep;
        double m_UnitRLTurn;

        double m_GoalFBStep;
        double m_GoalRLTurn;
        double m_FBStep;
        double m_RLTurn;
        double m_edge_dist_threshhold;
        double m_follow_threshhold;

        double m_right_x;
        double m_right_y;
        double m_left_x;
        double m_left_y;
        BallTracker m_BallTracker;
        ColorFinder m_BallFinder;

    protected:

    public:
        bool DEBUG_PRINT;
        int KickBall;        // 0: No ball 1:Left -1:Right

        GoalieBehavior();

        ~GoalieBehavior();

        void Process();

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);
    };
}