#pragma once


#include <list>
#include <map>
#include "configuration_strategy_t.h"

namespace drwn {
    class configuration_file_loader_t {
    public:
        static constexpr char DEFAULT_PATH[]{"res/config.ini"};

        configuration_file_loader_t() = default;

        void add_strategy(configuration_strategy_t& strategy, std::string path = "");

        void remove_strategy(const configuration_strategy_t& strategy);

        void configure_all();

        void dump_all();

        const std::string& get_default_path() const;

        void set_default_path(std::string path);

    private:
        static boost::property_tree::ptree read_property_from_file(const std::string& path);

        static void write_property_to_file(const boost::property_tree::ptree& prop, const std::string& path);

        std::map<std::string, std::list<configuration_strategy_t*>> get_file_associated_strategies() const;;

    private:
        bool m_debug{false};

        std::string m_default_path{DEFAULT_PATH};
        std::map<configuration_strategy_t*, std::string> m_strategies{};
    };
}


