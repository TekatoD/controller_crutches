#pragma once


#include "ConfigurationStrategy.h"

namespace Robot {
    class WalkingConfigurationStrategy
            : public ConfigurationStrategy {
    public:
        void ReadConfig(const boost::property_tree::ptree& prop) override;

        void WriteConfig(boost::property_tree::ptree& prop) const override;

    };
}


