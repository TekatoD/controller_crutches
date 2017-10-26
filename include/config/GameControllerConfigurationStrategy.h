/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#pragma once

#include "ConfigurationStrategy.h"

namespace Robot {
    class GameControllerConfigurationStrategy
            : public ConfigurationStrategy {
    public:

        static constexpr char DEFAULT_SECTION[] = "Game Controller";

        GameControllerConfigurationStrategy(std::string section = DEFAULT_SECTION);

        void ReadConfig(const boost::property_tree::ptree& prop) override;

        void WriteConfig(boost::property_tree::ptree& prop) const override;

    };
}