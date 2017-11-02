#include <log/trivial_logger_t.h>
#include <boost/filesystem.hpp>
#include "config/configuration_parser_t.h"
#include "config/ConfigurationFileLoader.h"

using namespace drwn;

void ConfigurationFileLoader::AddStrategy(configuration_strategy_t& strategy, std::string path) {
    if (path.empty()) {
        m_strategies.emplace(&strategy, m_default_path);
    } else {
        m_strategies.emplace(&strategy, std::move(path));
    }
}

void ConfigurationFileLoader::RemoveStrategy(const configuration_strategy_t& strategy) {
    m_strategies.erase((configuration_strategy_t*) &strategy);
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
                strategy->read_config(prop);
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
            strategy->write_config(prop);
        }
        WritePropertyToFile(prop, path);
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION FILE LOADER: Dump configuration to file " << path << " was done successfully";
    }
}

boost::property_tree::ptree ConfigurationFileLoader::ReadPropertyFromFile(const std::string& path) {
    return configuration_parser_t::read_from_file(path);
}

void ConfigurationFileLoader::WritePropertyToFile(const boost::property_tree::ptree& prop, const std::string& path) {
    configuration_parser_t::write_to_file(path, prop);
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

std::map<std::string, std::list<configuration_strategy_t*>> ConfigurationFileLoader::GetFileAssotiatedStrategies() const {
    namespace fs = boost::filesystem;
    std::map<std::string, std::list<configuration_strategy_t*>> result;
    for (auto& [strategy, path] : m_strategies) {
        auto it = result.find(path);
        if (it != result.end()) {
            auto& strategies = it->second;
            strategies.emplace_front(strategy);
        } else {
            result.emplace(path, std::list<configuration_strategy_t*>{strategy});
        }
    }
    return result;
}
