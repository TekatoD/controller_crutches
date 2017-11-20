#include <log/trivial_logger_t.h>
#include <boost/filesystem.hpp>
#include "config/configuration_parser_t.h"
#include "config/configuration_file_loader_t.h"

using namespace drwn;

void configuration_file_loader_t::add_strategy(configuration_strategy_t& strategy, std::string path) {
    if (path.empty()) {
        m_strategies.emplace(&strategy, m_default_path);
    } else {
        m_strategies.emplace(&strategy, std::move(path));
    }
}

void configuration_file_loader_t::remove_strategy(const configuration_strategy_t& strategy) {
    m_strategies.erase((configuration_strategy_t*) &strategy);
}

void configuration_file_loader_t::configure_all() {
    namespace fs = boost::filesystem;
    auto file_assotiated_strategied = get_file_associated_strategies();
    for (auto& [path, strategies] : file_assotiated_strategied) {
        if (fs::exists(fs::path(path))) {
            auto prop = read_property_from_file(path);
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

void configuration_file_loader_t::dump_all() {
    namespace fs = boost::filesystem;
    auto file_assotiated_strategied = get_file_associated_strategies();
    for (auto& [path, strategies] : file_assotiated_strategied) {
        boost::property_tree::ptree prop;
        if (fs::exists(fs::path(path))) {
            prop = read_property_from_file(path); // For avoid data loss in file
        }
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION FILE LOADER: Dumping configuration to file " << path << "...";
        for (auto& strategy : strategies) {
            strategy->write_config(prop);
        }
        write_property_to_file(prop, path);
        if (m_debug)
            LOG_DEBUG << "CONFIGURATION FILE LOADER: Dump configuration to file " << path << " was done successfully";
    }
}

boost::property_tree::ptree configuration_file_loader_t::read_property_from_file(const std::string& path) {
    return configuration_parser_t::read_from_file(path);
}

void configuration_file_loader_t::write_property_to_file(const boost::property_tree::ptree& prop,
                                                         const std::string& path) {
    configuration_parser_t::write_to_file(path, prop);
}

const std::string& configuration_file_loader_t::get_default_path() const {
    return m_default_path;
}

void configuration_file_loader_t::set_default_path(std::string path) {
    if (m_default_path != path) {
        for (auto& [strategy, cur_path] : m_strategies) {
            if (cur_path == m_default_path) {
                cur_path = path;
            }
        }
        m_default_path = path;
    }
}

std::map<std::string, std::list<configuration_strategy_t*>> configuration_file_loader_t::get_file_associated_strategies() const {
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

bool configuration_file_loader_t::is_debug_enabled() const {
    return m_debug;
}

void configuration_file_loader_t::enable_debug(bool debug) {
    m_debug = debug;
}
