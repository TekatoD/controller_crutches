/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#pragma once

#include <behavior/ball_tracker_t.h>
#include "config/configuration_strategy_t.h"

namespace drwn {
    class ball_tracker_configuration_strategy_t
            : public configuration_strategy_t {
    public:

        static constexpr char DEFAULT_SECTION[] = "Ball Tracker";

        explicit ball_tracker_configuration_strategy_t(std::string section = DEFAULT_SECTION);

        void read_config(const boost::property_tree::ptree& prop) override;

        void write_config(boost::property_tree::ptree& prop) const override;
    };
}