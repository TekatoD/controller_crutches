/**
 *  @autor tekatod
 *  @date 10/26/17
 */
#include <log/Logger.h>
#include "config/ConfigurationStrategy.h"

Robot::ConfigurationStrategy::ConfigurationStrategy(std::string section)
        : m_section(std::move(section)) { }

void Robot::ConfigurationStrategy::SetSection(std::string section) {
    if(m_debug) {
        LOG_DEBUG << "CONFIGURATION STRATEGY: section name = " << section;
    }
    m_section = std::move(section);
}

const std::string& Robot::ConfigurationStrategy::GetSection() const { return m_section; }

void Robot::ConfigurationStrategy::EnableDebug(bool debug) {
    m_debug = debug;
}

bool Robot::ConfigurationStrategy::IsDebugEnabled() const {
    return m_debug;
}
