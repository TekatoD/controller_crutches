/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#pragma once


#include <vector>
#include "DARwIn.h"


#define FINDER_SECTION "Ball Searcher"

namespace Robot {
    class BallSearcher {
    public:
        BallSearcher();

        void SetLastPosition(const Point2D& pos);

        void Process();

        void EnableWalking();

        void DisableWalking();

        bool IsWalkingEnabled() const;

        const Point2D& GetLastPosition() const;

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

    private:
        bool m_Active;
        Point2D m_LastPosition;

        double m_TiltPhase;
        double m_PanPhase;
        double m_TiltPhaseStep;
        double m_PanPhaseStep;
        double m_PhaseSize;

        double m_TurnSpeed;
        double m_TurnStep;
        double m_MaxTurn;

        int m_TurnDirection;
        int m_PanDirection;
        bool m_WalkingEnabled;
    };
}

