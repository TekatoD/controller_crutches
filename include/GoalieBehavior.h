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
#include "BallSearcher.h"

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

        int m_KickBallMaxCount;
        int m_KickBallCount;
        int m_PreviousState;

        float m_KickTopAngle;
        float m_KickRightAngle;

        float m_KickLeftAngle;
        float m_FollowMaxRL;
        float m_FitMaxRLTurn;

        float m_UnitRLTurn;
        float m_GoalRLTurn;
        float m_RLTurn;
        float m_EdgeDistThreshhold;
        float m_FollowThreshhold;

        BallSearcher m_BallSearcher;
        BallTracker m_BallTracker;
        ColorFinder m_BallFinder;
        Field m_Field;
        GoTo m_GoTo;

        float m_XCrutch;
        float m_ACrutch;
    };
}