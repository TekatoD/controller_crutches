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
    enum Kicking {
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
        double m_UnitRLStep;
        double m_UnitRLTurn;

        double m_GoalFBStep;
        double m_GoalRLStep;
        double m_GoalRLTurn;
        double m_FBStep;
        double m_RLStep;
        double m_RLTurn;

        double m_TiltOffset;

        double m_AimTiltOffset;
        double m_AimRLStep;
        double m_AimRLTurn;

        Kicking m_KickBall;

    public:

        BallFollower();

        ~BallFollower();

        void Process(Point2D ball_pos, double angle_top, double angle_bot);

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

        bool IsNoBall() const;

        Kicking GetKickingLeg() const;

    };
}

#endif
