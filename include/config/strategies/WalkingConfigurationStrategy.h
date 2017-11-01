#pragma once


#include "config/ConfigurationStrategy.h"

namespace drwn {
    class WalkingConfigurationStrategy
            : public ConfigurationStrategy {
    public:

        static constexpr char DEFAULT_SECTION[] = "Walking";

        explicit WalkingConfigurationStrategy(std::string section = DEFAULT_SECTION);

        void ReadConfig(const boost::property_tree::ptree& prop) override;

        void WriteConfig(boost::property_tree::ptree& prop) const override;

    };
}


