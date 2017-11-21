#include <log/trivial_logger_t.h>
#include "hw/LEDs_t.h"

using namespace drwn;

LEDs_t* LEDs_t::get_instance() {
    static LEDs_t instance;
    return &instance;
}

void LEDs_t::set_panel_led(bool first, bool second, bool third) {
    check_CM730();
    int mask = 0;
    if (first) mask |= 0x01;
    if (second) mask |= 0x02;
    if (third) mask |= 0x04;

    if (mask != m_mask) {
        m_cm730->write_byte(CM730_t::P_LED_PANNEL, mask, nullptr);
        m_mask = mask;
        if (m_debug) {
            LOG_DEBUG << "LEDS: LED panel was updated: "
                      << std::boolalpha << first << ' ' << second << ' ' << third;
        }
    }

}

void LEDs_t::set_head_led(const color_t& color) {
    check_CM730();
    if (m_head_color != color) {
        int value = CM730_t::make_color(color.get_red(), color.get_green(), color.get_blue());
        m_head_color = color;
        m_cm730->write_word(CM730_t::P_LED_HEAD_L, value, nullptr);
        if (m_debug) {
            LOG_DEBUG << "LEDS: Head LED color = ("
                      << color.get_red() << ' '
                      << color.get_green() << ' '
                      << color.get_blue() << ')';
        }
    }
}

void LEDs_t::set_eye_led(const color_t& color) {
    check_CM730();
    if (m_eye_color != color) {
        int value = CM730_t::make_color(color.get_red(), color.get_green(), color.get_blue());
        m_eye_color = color;
        m_cm730->write_word(CM730_t::P_LED_EYE_L, value, nullptr);
        if (m_debug) {
            LOG_DEBUG << "LEDS: Eye LED color = ("
                      << color.get_red() << ' '
                      << color.get_green() << ' '
                      << color.get_blue() << ')';
        }
    }
}

bool LEDs_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void LEDs_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

void LEDs_t::initialize(CM730_t* cm730) noexcept {
    m_cm730 = cm730;
}

void LEDs_t::check_CM730() {
    if (m_cm730 == nullptr) {
        if (m_debug) {
            LOG_ERROR << "LEDS: CM730_t isn't initialized";
        }
        throw std::runtime_error{"LEDs: CM730_t isn't initialized"};
    }
}
