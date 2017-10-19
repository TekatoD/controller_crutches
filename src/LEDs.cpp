#include <log/Logger.h>
#include "LEDs.h"

Robot::LEDs* Robot::LEDs::GetInstance() {
    static LEDs instance;
    return &instance;
}

void Robot::LEDs::SetPanelLed(bool first, bool second, bool third) {
    if (m_cm730 == nullptr) {LOG_ERROR << "LEDs: CM730 don't initialized";}


    int mask = 0;
    if (first) mask |= 0x01;
    if (second) mask |= 0x02;
    if (third) mask |= 0x04;
    m_cm730->WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, nullptr);
}

void Robot::LEDs::SetHeadLed(const Robot::Color& color) {
    int value = CM730::MakeColor(color.GetRed(), color.GetGreen(), color.GetBlue());
    m_cm730->WriteWord(CM730::P_LED_EYE_L, value, nullptr);
}

void Robot::LEDs::SetEyeLed(const Robot::Color& color) {
    int value = CM730::MakeColor(color.GetRed(), color.GetGreen(), color.GetBlue());
    m_cm730->WriteWord(CM730::P_LED_EYE_L, value, nullptr);
}

bool Robot::LEDs::IsDebugEnabled() const {
    return m_debug;
}

void Robot::LEDs::EnableDebug(bool debug) {
    m_debug = debug;
}

void Robot::LEDs::Initialize(Robot::CM730* cm730) {
    m_cm730 = cm730;
}
