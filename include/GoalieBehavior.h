/**
 *  @autor tekatod
 *  @date 5/5/17
 */
#pragma once

#include "math/Point.h"
#include "BallTracker.h"
#include "ColorFinder.h"
#include "Field.h"
#include "GoTo.h"

#define GOALIE_SECTION ("Goalie")

namespace Robot {
    class GoalieBehavior {
    public:

        GoalieBehavior();

        ~GoalieBehavior();

        void Process();

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

    protected:

        int m_NoBallMaxCount;
        int m_NoBallCount;
        int m_KickBallMaxCount;
        int m_KickBallCount;
        int m_PreviousState;

        double m_KickTopAngle;
        double m_KickRightAngle;

        double m_KickLeftAngle;
        double m_FollowMaxRL;
        double m_FitMaxRLTurn;

        double m_UnitRLTurn;
        double m_GoalRLTurn;
        double m_RLTurn;
        double m_EdgeDistThreshhold;

        double m_FollowThreshhold;
        BallTracker m_BallTracker;
        ColorFinder m_BallFinder;
        Field m_Field;
        GoTo m_GoTo;
    };
}