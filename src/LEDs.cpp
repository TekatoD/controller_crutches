#include <log/Logger.h>
#include "LEDs.h"

Robot::LEDs* Robot::LEDs::GetInstance() {
    static LEDs instance;
    return &instance;
}

void Robot::LEDs::SetPanelLed(bool first, bool second, bool third) {
    CheckCM730();
    int mask = 0;
    if (first) mask |= 0x01;
    if (second) mask |= 0x02;
    if (third) mask |= 0x04;
    m_cm730->WriteByte(CM730::P_LED_PANNEL, mask, nullptr);

    if (m_debug) {
        LOG_DEBUG << "LEDS: LED panel was updated: "
                  << std::boolalpha << first << ' ' << second << ' ' << third;
    }
}

void Robot::LEDs::SetHeadLed(const Robot::Color& color) {
    CheckCM730();
    int value = CM730::MakeColor(color.GetRed(), color.GetGreen(), color.GetBlue());
    m_cm730->WriteWord(CM730::P_LED_EYE_L, value, nullptr);
    if (m_debug) {
        LOG_DEBUG << "LEDS: Head LED color = ("
                  << color.GetRed() << ' ' << color.GetGreen() << ' ' << color.GetBlue() << ')';
    }
}

void Robot::LEDs::SetEyeLed(const Robot::Color& color) {
    CheckCM730();
    int value = CM730::MakeColor(color.GetRed(), color.GetGreen(), color.GetBlue());
    m_cm730->WriteWord(CM730::P_LED_EYE_L, value, nullptr);
    if (m_debug) {
        LOG_DEBUG << "LEDS: Eye LED color = ("
                  << color.GetRed() << ' ' << color.GetGreen() << ' ' << color.GetBlue() << ')';
    }
}

bool Robot::LEDs::IsDebugEnabled() const noexcept {
    return m_debug;
}

void Robot::LEDs::EnableDebug(bool debug) noexcept {
    m_debug = debug;
}

void Robot::LEDs::Initialize(Robot::CM730* cm730) noexcept {
    m_cm730 = cm730;
}

void Robot::LEDs::CheckCM730() {
    if (m_cm730 == nullptr) {
        if (m_debug) {
            LOG_ERROR << "LEDS: CM730 don't initialized";
        }
        throw std::runtime_error{"LEDs: CM730 don't initialized"};
    }
}
