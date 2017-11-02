#pragma once


#include <boost/property_tree/ptree.hpp>

namespace drwn {
    class configuration_strategy_t {
    public:

        explicit configuration_strategy_t(std::string section = "");

        virtual void read_config(const boost::property_tree::ptree& prop) = 0;

        virtual void write_config(boost::property_tree::ptree& prop) const = 0;

        const std::string& get_section() const;

        void set_section(std::string section);

        void enable_debug(bool debug);

        bool is_debug_enabled() const;

        virtual ~configuration_strategy_t() = default;

    private:
        std::string m_section{};
        bool m_debug{false};
    };
}


