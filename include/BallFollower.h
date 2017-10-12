/*
 *   BallFollower.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _BALL_FOLLOWER_H_
#define _BALL_FOLLOWER_H_

#include "math/Point.h"
#include "BallTracker.h"

#define FOLLOWER_SECTION ("Ball Follower")


namespace Robot {
    enum KickingAction {
        NO_KICKING,
        LEFT_LEG_KICK,
        RIGHT_LEG_KICK
    };

    class BallFollower {
    private:
        int m_NoBallMaxCount;
        int m_NoBallCount;
        int m_KickBallMaxCount;
        int m_KickBallCount;

        float m_MaxFBStep;
        float m_MaxRLStep;
        float m_MaxDirAngle;

        float m_KickTopAngle;
        float m_KickRightAngle;
        float m_KickLeftAngle;

        float m_FollowMaxFBStep;
        float m_FollowMinFBStep;
        float m_FollowMaxRLTurn;
        float m_FitFBStep;
        float m_FitMaxRLTurn;
        float m_UnitFBStep;
        float m_UnitRLStep;
        float m_UnitRLTurn;

        float m_GoalFBStep;
        float m_GoalRLStep;
        float m_GoalRLTurn;
        float m_FBStep;
        float m_RLStep;
        float m_RLTurn;

        float m_TiltOffset;

        float m_AimTiltOffset;
        float m_AimRLStep;
        float m_AimRLTurn;

        KickingAction m_KickBall;

    public:

        BallFollower();

        ~BallFollower();

        void Process(Point2D ball_pos, float angle_top, float angle_bot);

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

        bool IsNoBall() const;

        KickingAction GetKickingLeg() const;

    };
}

#endif
