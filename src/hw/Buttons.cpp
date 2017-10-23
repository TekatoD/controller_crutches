#include "hw/Buttons.h"
#include <stdexcept>
#include <motion/MotionStatus.h>
#include <log/Logger.h>


Robot::Buttons* Robot::Buttons::GetInstance() {
    static Buttons instance;
    return &instance;
}

void Robot::Buttons::Update() {
    m_button_status[FIRST_BUTTON] = (bool) MotionStatus::BUTTON & (FIRST_BUTTON + 1);
    m_button_status[SECOND_BUTTON] = (bool) MotionStatus::BUTTON & (SECOND_BUTTON + 1);
    if (m_debug) {
        LOG_DEBUG << std::boolalpha
                  << "BUTTONS: first_button = " << m_button_status[FIRST_BUTTON]
                  << ", second_button = " << m_button_status[SECOND_BUTTON];
    }
}

bool Robot::Buttons::IsButtonPressed(Robot::Buttons::ButtonId btn) const {
    return m_button_status[btn];
}

bool Robot::Buttons::IsDebugEnabled() const noexcept {
    return m_debug;
}

void Robot::Buttons::EnableDebug(bool debug) noexcept {
    m_debug = debug;
}
