#include <log/trivial_logger_t.h>
#include "hw/LEDs_t.h"

drwn::LEDs_t* drwn::LEDs_t::get_instance() {
    static LEDs_t instance;
    return &instance;
}

void drwn::LEDs_t::set_panel_led(bool first, bool second, bool third) {
    check_CM730();
    int mask = 0;
    if (first) mask |= 0x01;
    if (second) mask |= 0x02;
    if (third) mask |= 0x04;
    m_cm730->write_byte(CM730_t::P_LED_PANNEL, mask, nullptr);

    if (m_debug) {
        LOG_DEBUG << "LEDS: LED panel was updated: "
                  << std::boolalpha << first << ' ' << second << ' ' << third;
    }
}

void drwn::LEDs_t::set_head_led(const drwn::color_t& color) {
    check_CM730();
    int value = CM730_t::make_color(color.get_red(), color.get_green(), color.get_blue());
    m_cm730->write_word(CM730_t::P_LED_EYE_L, value, nullptr);
    if (m_debug) {
        LOG_DEBUG << "LEDS: head_t LED color = ("
                  << color.get_red() << ' ' << color.get_green() << ' ' << color.get_blue() << ')';
    }
}

void drwn::LEDs_t::set_eye_led(const drwn::color_t& color) {
    check_CM730();
    int value = CM730_t::make_color(color.get_red(), color.get_green(), color.get_blue());
    m_cm730->write_word(CM730_t::P_LED_EYE_L, value, nullptr);
    if (m_debug) {
        LOG_DEBUG << "LEDS: Eye LED color = ("
                  << color.get_red() << ' ' << color.get_green() << ' ' << color.get_blue() << ')';
    }
}

bool drwn::LEDs_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void drwn::LEDs_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

void drwn::LEDs_t::initialize(drwn::CM730_t* cm730) noexcept {
    m_cm730 = cm730;
}

void drwn::LEDs_t::check_CM730() {
    if (m_cm730 == nullptr) {
        if (m_debug) {
            LOG_ERROR << "LEDS: CM730_t isn't initialized";
        }
        throw std::runtime_error{"LEDs: CM730_t isn't initialized"};
    }
}
