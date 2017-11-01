/**
 *  @autor tekatod
 *  @date 10/26/17
 */
#include <log/trivial_logger_t.h>
#include "config/ConfigurationStrategy.h"

drwn::ConfigurationStrategy::ConfigurationStrategy(std::string section)
        : m_section(std::move(section)) { }

void drwn::ConfigurationStrategy::SetSection(std::string section) {
    if(m_debug) {
        LOG_DEBUG << "CONFIGURATION STRATEGY: section name = " << section;
    }
    m_section = std::move(section);
}

const std::string& drwn::ConfigurationStrategy::GetSection() const { return m_section; }

void drwn::ConfigurationStrategy::EnableDebug(bool debug) {
    m_debug = debug;
}

bool drwn::ConfigurationStrategy::IsDebugEnabled() const {
    return m_debug;
}
