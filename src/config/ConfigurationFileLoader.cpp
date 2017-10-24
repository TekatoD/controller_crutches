#include <log/Logger.h>
#include <config/ConfigurationParser.h>
#include "config/ConfigurationFileLoader.h"

using namespace Robot;

void ConfigurationFileLoader::AddStrategy(std::string section,
                                      Robot::ConfigurationStrategy* strategy,
                                      std::string path) {
    if (path.empty()) {
        m_section_nodes.emplace(std::move(section), SectionNode{strategy, m_default_path});
    } else {
        m_section_nodes.emplace(std::move(section), SectionNode{strategy, std::move(path)});
    }
}

void ConfigurationFileLoader::RemoveStrategy(const std::string& section) {
    m_section_nodes.erase(section);
}

void ConfigurationFileLoader::ConfigureSection(const std::string& section) {
    auto it = m_section_nodes.find(section);
    if (it != m_section_nodes.end()) {
        auto& node = it->second;
        auto& path = node.path;
        auto& strategy = node.strategy;
        auto prop = ReadFile(path);
        strategy->ReadConfig(prop);
    } else {
        throw std::runtime_error{"Can't read strategy for section: " + section};
    }
}

void ConfigurationFileLoader::ConfigureAll() {
    for (auto& pair : m_section_nodes) {
        auto& section = pair.first;
        auto& node = pair.second;
        auto& strategy = node.strategy;
        auto& path = node.path;
        auto prop = ReadFile(path);
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION READER: Configuring section " << section << "...";
        strategy->ReadConfig(prop);
        if (m_debug)
            LOG_INFO << "CONFIGURATION READER: Configuration for section " << section << " was done successfully";
    }
}

void ConfigurationFileLoader::DumpAll() {
    throw std::runtime_error{"DumpAll isn't implemented"};
}

const std::string& ConfigurationFileLoader::GetSectionConfigFile(const std::string& section) {
    auto it = m_section_nodes.find(section);
    if (it != m_section_nodes.end()) {
        auto& node = it->second;
        return node.path;
    } else {
        throw std::runtime_error("Can't set config file for section: " + section);
    }
}

void ConfigurationFileLoader::SetSectionConfigFile(const std::string& section, std::string path) {
    auto it = m_section_nodes.find(section);
    if (it != m_section_nodes.end()) {
        auto& node = it->second;
        node.path = std::move(path);
    } else {
        throw std::runtime_error("Can't set config file for section: " + section);
    }
}

boost::property_tree::ptree ConfigurationFileLoader::ReadFile(const std::string& path) {
    return ConfigurationParser::ReadFromFile(path);
}

void ConfigurationFileLoader::WriteFile(const boost::property_tree::ptree& ptree, const std::string& path) {
    throw std::runtime_error{"WriteFile isn't implemented"};
}

const std::string& ConfigurationFileLoader::GetDefaultPath() const {
    return m_default_path;
}

void ConfigurationFileLoader::SetDefaultPath(std::string path) {
    if (m_default_path != path) {
        for (auto& pair : m_section_nodes) {
            auto& node = pair.second;
            if (node.path == m_default_path) {
                node.path = path;
            }
        }
        m_default_path = path;
    }
}
