//
// Created by akovalev on 16.11.17.
//

#pragma once

#include <localization/localization_util_t.h>
#include <config/configuration_strategy_t.h>

namespace drwn {
    class localization_field_configuration_strategy_t : public configuration_strategy_t {
    public:
        static constexpr char DEFAULT_SECTION[] = "localization_field";

        explicit localization_field_configuration_strategy_t(localization::field_map_t* field_ptr, std::string section = DEFAULT_SECTION);

        void read_config(const boost::property_tree::ptree& prop) override;
        void write_config(boost::property_tree::ptree& prop) const override;

        void set_field_map(localization::field_map_t* field_ptr);
        localization::field_map_t* get_field_map() const;
    private:
        localization::field_map_t* m_field_map;
    };
}

