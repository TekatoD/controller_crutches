/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#pragma once


#include <vector>
#include <math/point_t.h>

#define FINDER_SECTION "Ball Searcher"

namespace drwn {
    class BallSearcher {
    public:
        BallSearcher();

        void SetLastPosition(const point_2D_t& pos);

        void Process();

        void EnableWalking();

        void DisableWalking();

        bool IsWalkingEnabled() const;

        const point_2D_t& GetLastPosition() const;

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

        void EnableDebug(bool debug);

        bool IsDebugEnabled() const;


    private:
        bool m_debug{false};
        bool m_Active;
        point_2D_t m_LastPosition;

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

