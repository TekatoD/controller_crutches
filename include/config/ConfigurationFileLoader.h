#pragma once


#include <unordered_map>
#include <list>
#include <map>
#include "ConfigurationStrategy.h"

namespace Robot {
    struct ConfigurationFileLoaderNode {
        ConfigurationStrategy* strategy{nullptr};
        std::string path{};
    };

    class ConfigurationFileLoader {
    public:
        static constexpr char DEFAULT_PATH[]{"res/config.ini"};

        ConfigurationFileLoader() = default;

        void AddStrategy(std::string section, ConfigurationStrategy* strategy, std::string path = "");

        void RemoveStrategy(const std::string& section);

        void ConfigureSection(const std::string& section);

        void ConfigureAll();

        void DumpAll();

        const std::string& GetSectionConfigFile(const std::string& section);

        void SetSectionConfigFile(const std::string& section, std::string path);

        const std::string& GetDefaultPath() const;

        void SetDefaultPath(std::string path);

    private:
        static boost::property_tree::ptree ReadPropertyFromFile(const std::string& path);

        static void WritePropertyToFile(const boost::property_tree::ptree& prop, const std::string& path);

        std::map<std::string, std::list<ConfigurationStrategy*>> GetFileAssotiatedStrategies() const;;

    private:
        bool m_debug{true};

        std::string m_default_path{DEFAULT_PATH};
        std::unordered_map<std::string, ConfigurationFileLoaderNode> m_section_nodes{};
    };
}


