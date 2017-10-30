#include <log/Logger.h>
#include <boost/filesystem.hpp>
#include "config/ConfigurationParser.h"
#include "config/ConfigurationFileLoader.h"

using namespace Robot;

void ConfigurationFileLoader::AddStrategy(ConfigurationStrategy& strategy, std::string path) {
    if (path.empty()) {
        m_strategies.emplace(&strategy, m_default_path);
    } else {
        m_strategies.emplace(&strategy, std::move(path));
    }
}

void ConfigurationFileLoader::RemoveStrategy(const ConfigurationStrategy& strategy) {
    m_strategies.erase((ConfigurationStrategy*) &strategy);
}

void ConfigurationFileLoader::ConfigureAll() {
    namespace fs = boost::filesystem;
    auto file_assotiated_strategied = GetFileAssotiatedStrategies();
    for (auto& [path, strategies] : file_assotiated_strategied) {
        if (fs::exists(fs::path(path))) {
            auto prop = ReadPropertyFromFile(path);
            if (m_debug)
                LOG_DEBUG << "CONFIGURATION FILE LOADER: Reading configuration from file " << path << "...";
            for (auto& strategy : strategies) {
                strategy->ReadConfig(prop);
            }
            if (m_debug)
                LOG_DEBUG << "CONFIGURATION FILE LOADER: Reading configuration from file " << path << " was done successfully";
        } else {
           if (m_debug) LOG_WARNING << "CONFIGURATION FILE LOADER: File " << path << " doesn't exist";
        }
    }
}

void ConfigurationFileLoader::DumpAll() {
    namespace fs = boost::filesystem;
    auto file_assotiated_strategied = GetFileAssotiatedStrategies();
    for (auto& [path, strategies] : file_assotiated_strategied) {
        boost::property_tree::ptree prop;
        if (fs::exists(fs::path(path))) {
            prop = ReadPropertyFromFile(path); // For avoid data loss in file
        }
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION FILE LOADER: Dumping configuration to file " << path << "...";
        for (auto& strategy : strategies) {
            strategy->WriteConfig(prop);
        }
        WritePropertyToFile(prop, path);
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION FILE LOADER: Dump configuration to file " << path << " was done successfully";
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
        for (auto& [strategy, cur_path] : m_strategies) {
            if (cur_path == m_default_path) {
                cur_path = path;
            }
        }
        m_default_path = path;
    }
}

std::map<std::string, std::list<ConfigurationStrategy*>> ConfigurationFileLoader::GetFileAssotiatedStrategies() const {
    namespace fs = boost::filesystem;
    std::map<std::string, std::list<ConfigurationStrategy*>> result;
    for (auto& [strategy, path] : m_strategies) {
        auto it = result.find(path);
        if (it != result.end()) {
            auto& strategies = it->second;
            strategies.emplace_front(strategy);
        } else {
            result.emplace(path, std::list<ConfigurationStrategy*>{strategy});
        }
    }
    return result;
}
