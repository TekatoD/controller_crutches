//
// Created by akovalev on 16.11.17.
//

#pragma once

#include <localization/line_t.h>
#include <config/configuration_strategy_t.h>
#include <localization/field_map_t.h>

namespace drwn {
    class localization_field_configuration_strategy_t : public configuration_strategy_t {
    public:
        static constexpr char DEFAULT_SECTION[] = "Field";

        explicit localization_field_configuration_strategy_t(std::string section = DEFAULT_SECTION);

        void read_config(const boost::property_tree::ptree& prop) override;

        void write_config(boost::property_tree::ptree& prop) const override;

    private:
    };
}

