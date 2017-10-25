#include <log/Logger.h>
#include <config/ConfigurationParser.h>
#include "config/ConfigurationFileLoader.h"

using namespace Robot;

void ConfigurationFileLoader::AddStrategy(std::string section,
                                          ConfigurationStrategy& strategy,
                                          std::string path) {
    if (path.empty()) {
        m_section_nodes.emplace(std::move(section), ConfigurationFileLoaderNode{&strategy, m_default_path});
    } else {
        m_section_nodes.emplace(std::move(section), ConfigurationFileLoaderNode{&strategy, std::move(path)});
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
        auto prop = ReadPropertyFromFile(path);
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION READER: Configuring section " << section << " from file " << path << "...";
        strategy->ReadConfig(prop);
    } else {
        throw std::runtime_error{"Can't read strategy for section: " + section};
    }
}

void ConfigurationFileLoader::ConfigureAll() {
    auto file_assotiated_strategied = GetFileAssotiatedStrategies();
    for (auto& [path, strategies] : file_assotiated_strategied) {
        auto prop = ReadPropertyFromFile(path);
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION READER: Reading configuration from file " << path << "...";
        for (auto& strategy : strategies) {
            strategy->ReadConfig(prop);
        }
        if (m_debug)
            LOG_INFO << "CONFIGURATION READER: Read configuration from file " << path << " was done successfully";
    }
}

void ConfigurationFileLoader::DumpAll() {
    auto file_assotiated_strategied = GetFileAssotiatedStrategies();
    for (auto& [path, strategies] : file_assotiated_strategied) {
        boost::property_tree::ptree prop;
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION READER: Dumping configuration to file " << path << "...";
        for (auto& strategy : strategies) {
            strategy->WriteConfig(prop);
        }
        WritePropertyToFile(prop, path);
        if (m_debug)
            LOG_INFO << "CONFIGURATION READER: Dump configuration to file " << path << " was done successfully";
    }
}

const std::string& ConfigurationFileLoader::GetSectionConfigFile(const std::string& section) {
    auto it = m_section_nodes.find(section);
    if (it != m_section_nodes.end()) {
        const auto& node = it->second;
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

boost::property_tree::ptree ConfigurationFileLoader::ReadPropertyFromFile(const std::string& path) {
    return ConfigurationParser::ReadFromFile(path);
}

void ConfigurationFileLoader::WritePropertyToFile(const boost::property_tree::ptree& prop, const std::string& path) {
    ConfigurationParser::WriteToFile(path, prop);
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

std::map<std::string, std::list<ConfigurationStrategy*>> ConfigurationFileLoader::GetFileAssotiatedStrategies() const {
    std::map<std::string, std::list<ConfigurationStrategy*>> result;
    for (auto& [section, node] : m_section_nodes) {
        const auto& path = node.path;
        const auto& strategy = node.strategy;

        auto it = result.find(path);
        if (it != result.end()) {
            auto& strategies = it->second;
            strategies.emplace_front(strategy);
        } else {
            result.emplace(path, std::list<ConfigurationStrategy*>{strategy});
        }
    }
}
