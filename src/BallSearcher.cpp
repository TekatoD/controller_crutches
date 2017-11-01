/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#include <Camera.h>
#include <motion/modules/head_t.h>
#include <cmath>
#include <motion/modules/walking_t.h>
#include <log/trivial_logger_t.h>
#include "BallSearcher.h"

using namespace drwn;

BallSearcher::BallSearcher() {
    m_Active = false;

    m_TiltPhase = 0.0;
    m_PanPhase = 0.0;
    m_TiltPhaseStep = 1.0;
    m_PanPhaseStep = 2.0;
    m_PhaseSize = 500.0;

    m_TurnSpeed = 0.0;
    m_TurnStep = 1.0;
    m_MaxTurn = 20.0;

    m_TurnDirection = 0;
    m_PanDirection = 0;

    m_WalkingEnabled = true;
    m_LastPosition = point_2D_t(Camera::WIDTH / 2, Camera::HEIGHT / 2);
}

void BallSearcher::Process() {
   if (!m_Active) {
       // TODO Pan checking
        point_2D_t center = point_2D_t(Camera::WIDTH / 2, Camera::HEIGHT / 2);
        point_2D_t offset = m_LastPosition - center;

        m_PanPhase = 0.0;
        m_TiltPhase = 0.0;

        m_PanDirection = offset.X > 0 ? 1 : -1;
        m_TurnDirection = offset.Y > 0 ? 1 : -1;

        m_Active = true;
    }


    m_PanPhase += m_PanPhaseStep * m_PanDirection;
    m_TiltPhase += m_TiltPhaseStep * m_TurnDirection;

    const float tilt_max = head_t::GetInstance()->get_top_limit_angle();
    const float tilt_min = head_t::GetInstance()->get_bottom_limit_angle();
    const float tilt_diff = tilt_max - tilt_min;
    const float pan_max = head_t::GetInstance()->get_left_limit_angle();
    const float pan_min = head_t::GetInstance()->get_right_limit_angle();
    const float pan_diff = pan_max - pan_min;



    float pan = sinf(m_PanPhase / m_PhaseSize * M_2_PI) * pan_diff - pan_min;
    float tilt = sinf(m_TiltPhase / m_PhaseSize * M_2_PI) * tilt_diff - tilt_min;
    head_t::GetInstance()->move_by_angle(pan, tilt);

    if (m_WalkingEnabled) {
        m_TurnSpeed = walking_t::GetInstance()->get_a_move_amplitude();
        m_TurnSpeed += m_TurnStep * m_TurnDirection;
        if (fabsf(m_TurnSpeed) > m_MaxTurn) {
            m_TurnSpeed = m_MaxTurn * m_TurnDirection;
        }

        walking_t::GetInstance()->set_x_move_amplitude(0);
        walking_t::GetInstance()->set_x_move_amplitude(0);
        walking_t::GetInstance()->set_x_move_amplitude(m_TurnSpeed);
        walking_t::GetInstance()->set_move_aim_on(false);
        walking_t::GetInstance()->start();
    } else {
        walking_t::GetInstance()->stop();
    }
}

void BallSearcher::EnableWalking() {
    m_WalkingEnabled = true;
}

void BallSearcher::DisableWalking() {
    m_WalkingEnabled = false;
}

bool BallSearcher::IsWalkingEnabled() const {
    return m_WalkingEnabled;
}

const point_2D_t& BallSearcher::GetLastPosition() const {
    return m_LastPosition;
}

void BallSearcher::SetLastPosition(const point_2D_t& pos) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: last_position_x = " << pos.X << " last_position_y = " << pos.Y;
    }
    if (pos.X != -1 && pos.Y != -1) {
        m_LastPosition = pos;
        m_Active = false;
    }
}

float BallSearcher::GetTiltPhaseStep() const {
    return m_TiltPhaseStep;
}

void BallSearcher::SetTiltPhaseStep(float tilt_phase_step) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: tilt_phase_step = " << tilt_phase_step;
    }
    m_TiltPhaseStep = tilt_phase_step;
}

float BallSearcher::GetPanPhaseStep() const {
    return m_PanPhaseStep;
}

void BallSearcher::SetPanPhaseStep(float pan_phase_step) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: pan_phase_step = " << pan_phase_step;
    }
    m_PanPhaseStep = pan_phase_step;
}

float BallSearcher::GetPhaseSize() const {
    return m_PhaseSize;
}

void BallSearcher::SetPhaseSize(float phase_size) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: phase_size = " << phase_size;
    }
    m_PhaseSize = phase_size;
}

float BallSearcher::GetTurnStep() const {
    return m_TurnStep;
}

void BallSearcher::SetTurnStep(float turn_step) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: turn_step = " << turn_step;
    }
    m_TurnStep = turn_step;
}

float BallSearcher::GetMaxTurn() const {
    return m_MaxTurn;
}

void BallSearcher::SetMaxTurn(float max_turn) {
    if(m_debug) {
        LOG_DEBUG << "BALL SEARCHER: max_turn = " << max_turn;
    }
    m_MaxTurn = max_turn;
}

void BallSearcher::EnableDebug(bool debug) { m_debug = debug; }

bool BallSearcher::IsDebugEnabled() const { return m_debug; }
