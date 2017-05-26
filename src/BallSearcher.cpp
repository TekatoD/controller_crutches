/**
 *  @autor arssivka
 *  @date 5/11/17
 */

#include "BallSearcher.h"

using namespace Robot;

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
    m_LastPosition = Point2D(Camera::WIDTH / 2, Camera::HEIGHT / 2);
}

void BallSearcher::Process() {
   if (!m_Active) {
       // TODO Pan checking
        Point2D center = Point2D(Camera::WIDTH / 2, Camera::HEIGHT / 2);
        Point2D offset = m_LastPosition - center;

        m_PanPhase = 0.0;
        m_TiltPhase = 0.0;

        m_PanDirection = offset.X > 0 ? 1 : -1;
        m_TurnDirection = offset.Y > 0 ? 1 : -1;

        m_Active = true;
    }


    m_PanPhase += m_PanPhaseStep * m_PanDirection;
    m_TiltPhase += m_TiltPhaseStep * m_TurnDirection;

    const double tilt_max = Head::GetInstance()->GetTopLimitAngle();
    const double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
    const double tilt_diff = tilt_max - tilt_min;
    const double pan_max = Head::GetInstance()->GetLeftLimitAngle();
    const double pan_min = Head::GetInstance()->GetRightLimitAngle();
    const double pan_diff = pan_max - pan_min;



    double pan = sin(m_PanPhase / m_PhaseSize * M_2_PI) * pan_diff - pan_min;
    double tilt = sin(m_TiltPhase / m_PhaseSize * M_2_PI) * tilt_diff - tilt_min;
    Head::GetInstance()->MoveByAngle(pan, tilt);

    if (m_WalkingEnabled) {
        m_TurnSpeed = Walking::GetInstance()->A_MOVE_AMPLITUDE;
        m_TurnSpeed += m_TurnStep * m_TurnDirection;
        if (fabs(m_TurnSpeed) > m_MaxTurn) {
            m_TurnSpeed = m_MaxTurn * m_TurnDirection;
        }

        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
        Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = m_TurnSpeed;
        Walking::GetInstance()->A_MOVE_AIM_ON = false;
        Walking::GetInstance()->Start();
    } else {
        Walking::GetInstance()->Stop();
    }
}

void BallSearcher::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, FINDER_SECTION);
}

void BallSearcher::LoadINISettings(minIni* ini, const std::string& section) {
    double value;
    if ((value = ini->getd(section, "tilt_phase_step", INVALID_VALUE)) != INVALID_VALUE) m_TiltPhaseStep = value;
    if ((value = ini->getd(section, "pan_phase_step", INVALID_VALUE)) != INVALID_VALUE) m_PanPhaseStep = value;
    if ((value = ini->getd(section, "phase_size", INVALID_VALUE)) != INVALID_VALUE) m_PhaseSize = value;
    if ((value = ini->getd(section, "turn_step", INVALID_VALUE)) != INVALID_VALUE) m_TurnStep = value;
    if ((value = ini->getd(section, "max_turn", INVALID_VALUE)) != INVALID_VALUE) m_MaxTurn = value;
}

void BallSearcher::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, FINDER_SECTION);
}

void BallSearcher::SaveINISettings(minIni* ini, const std::string& section) {
    ini->put(section, "tilt_phase_step", m_TiltPhaseStep);
    ini->put(section, "pan_phase_step", m_PanPhaseStep);
    ini->put(section, "phase_size", m_PhaseSize);
    ini->put(section, "turn_step", m_TurnStep);
    ini->put(section, "max_turn", m_MaxTurn);
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

const Point2D& BallSearcher::GetLastPosition() const {
    return m_LastPosition;
}

void BallSearcher::SetLastPosition(const Point2D& pos) {
    if (pos.X != -1 && pos.Y != -1) {
        m_LastPosition = pos;
        m_Active = false;
    }
}
