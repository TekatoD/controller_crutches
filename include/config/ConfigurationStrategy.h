#pragma once


#include <boost/property_tree/ptree.hpp>

namespace Robot {
    class ConfigurationStrategy {
    public:

        explicit ConfigurationStrategy(std::string section = "");

        virtual void ReadConfig(const boost::property_tree::ptree& prop) = 0;

        virtual void WriteConfig(boost::property_tree::ptree& prop) const = 0;

        const std::string& GetSection() const;

        void SetSection(std::string section);

        virtual ~ConfigurationStrategy() = default;

    private:
        std::string m_section{};
    };
}


