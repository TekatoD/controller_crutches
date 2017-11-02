/**
 *  @autor tekatod
 *  @date 10/26/17
 */
#include <log/trivial_logger_t.h>
#include "config/configuration_strategy_t.h"

drwn::configuration_strategy_t::configuration_strategy_t(std::string section)
        : m_section(std::move(section)) { }

void drwn::configuration_strategy_t::set_section(std::string section) {
    if(m_debug) {
        LOG_DEBUG << "CONFIGURATION STRATEGY: section name = " << section;
    }
    m_section = std::move(section);
}

const std::string& drwn::configuration_strategy_t::get_section() const { return m_section; }

void drwn::configuration_strategy_t::enable_debug(bool debug) {
    m_debug = debug;
}

bool drwn::configuration_strategy_t::is_debug_enabled() const {
    return m_debug;
}
