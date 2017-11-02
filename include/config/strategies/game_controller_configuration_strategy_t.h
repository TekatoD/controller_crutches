/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#pragma once

#include "config/configuration_strategy_t.h"

namespace drwn {
    class game_controller_configuration_strategy_t
            : public configuration_strategy_t {
    public:

        static constexpr char DEFAULT_SECTION[] = "Game Controller";

        explicit game_controller_configuration_strategy_t(std::string section = DEFAULT_SECTION);

        void read_config(const boost::property_tree::ptree& prop) override;

        void write_config(boost::property_tree::ptree& prop) const override;

    };
}