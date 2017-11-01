#include "hw/buttons_t.h"
#include <stdexcept>
#include <motion/motion_status_t.h>
#include <log/trivial_logger_t.h>


drwn::buttons_t* drwn::buttons_t::get_instance() {
    static buttons_t instance;
    return &instance;
}

void drwn::buttons_t::update() {
    m_button_status[FIRST_BUTTON] = (bool) motion_status_t::BUTTON & (FIRST_BUTTON + 1);
    m_button_status[SECOND_BUTTON] = (bool) motion_status_t::BUTTON & (SECOND_BUTTON + 1);
    if (m_debug) {
        LOG_DEBUG << std::boolalpha
                  << "BUTTONS: first_button = " << m_button_status[FIRST_BUTTON]
                  << ", second_button = " << m_button_status[SECOND_BUTTON];
    }
}

bool drwn::buttons_t::is_button_pressed(drwn::buttons_t::button_id btn) const {
    return m_button_status[btn];
}

bool drwn::buttons_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void drwn::buttons_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}
