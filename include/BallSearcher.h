/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#pragma once


#include <vector>
#include <math/Point.h>

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

        float GetTiltPhaseStep() const;

        void SetTiltPhaseStep(float tilt_phase_step);

        float GetPanPhaseStep() const;

        void SetPanPhaseStep(float pan_phase_step);

        float GetPhaseSize() const;

        void SetPhaseSize(float phase_size);

        float GetTurnStep() const;

        void SetTurnStep(float turn_step);

        float GetMaxTurn() const;

        void SetMaxTurn(float max_turn);


    private:
        bool m_Active;
        Point2D m_LastPosition;

        float m_TiltPhase;
        float m_PanPhase;
        float m_TiltPhaseStep;
        float m_PanPhaseStep;
        float m_PhaseSize;

        float m_TurnSpeed;
        float m_TurnStep;
        float m_MaxTurn;

        int m_TurnDirection;
        int m_PanDirection;
        bool m_WalkingEnabled;
    };
}

