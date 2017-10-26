/**
 *  @autor tekatod
 *  @date 10/26/17
 */
#include "config/ConfigurationStrategy.h"

Robot::ConfigurationStrategy::ConfigurationStrategy(std::string section)
        : m_section(std::move(section)) { }

void Robot::ConfigurationStrategy::SetSection(std::string section) { m_section = std::move(section); }

const std::string& Robot::ConfigurationStrategy::GetSection() const { return m_section; }
