//
// Created by akovalev on 16.11.17.
//

#include <config/strategies/localization_field_configuration_strategy_t.h>

using namespace drwn;

localization_field_configuration_strategy_t::localization_field_configuration_strategy_t(field_map_t* field_ptr,
                                                                                         std::string section)
        : configuration_strategy_t(std::move(section)), m_field_map(field_ptr) {
}

void localization_field_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (m_field_map == nullptr) {
        throw std::runtime_error{"Localization configuration load failure: m_field_map nullptr"};
    }

    if (prop.count(this->get_section()) == 0) { return; }

    auto& section = prop.get_child(this->get_section());

    auto field_width = section.get_optional<float>("field_width");
    auto field_height = section.get_optional<float>("field_height");
    auto gate_height = section.get_optional<float>("gate_height");
    auto penalty_width = section.get_optional<float>("penalty_width");
    auto penalty_height = section.get_optional<float>("penalty_height");

    if (field_width) { m_field_map->set_field_width(field_width.get()); }
    if (field_height) { m_field_map->set_field_height(field_height.get()); }
    if (gate_height) { m_field_map->set_gate_height(gate_height.get()); }
    if (penalty_width) { m_field_map->set_penalty_width(penalty_width.get()); }
    if (penalty_height) { m_field_map->set_penalty_height(penalty_height.get()); }
}

void localization_field_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    //TODO: :^)
}

void localization_field_configuration_strategy_t::set_field_map(drwn::field_map_t* field_ptr) {
    m_field_map = field_ptr;
}

drwn::field_map_t* localization_field_configuration_strategy_t::get_field_map() const {
    return m_field_map;
}

