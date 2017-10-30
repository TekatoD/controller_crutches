#pragma once


#include <list>
#include <map>
#include "ConfigurationStrategy.h"

namespace Robot {
    class ConfigurationFileLoader {
    public:
        static constexpr char DEFAULT_PATH[]{"res/config.ini"};

        ConfigurationFileLoader() = default;

        void AddStrategy(ConfigurationStrategy& strategy, std::string path = "");

        void RemoveStrategy(const ConfigurationStrategy& strategy);

        void ConfigureAll();

        void DumpAll();

        const std::string& GetDefaultPath() const;

        void SetDefaultPath(std::string path);

    private:
        static boost::property_tree::ptree ReadPropertyFromFile(const std::string& path);

        static void WritePropertyToFile(const boost::property_tree::ptree& prop, const std::string& path);

        std::map<std::string, std::list<ConfigurationStrategy*>> GetFileAssotiatedStrategies() const;;

    private:
        bool m_debug{false};

        std::string m_default_path{DEFAULT_PATH};
        std::map<ConfigurationStrategy*, std::string> m_strategies{};
    };
}


