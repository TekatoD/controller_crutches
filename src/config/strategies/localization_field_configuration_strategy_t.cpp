//
// Created by akovalev on 16.11.17.
//

#include <config/strategies/localization_field_configuration_strategy_t.h>
#include <math/angle_tools.h>

using namespace drwn;

localization_field_configuration_strategy_t::localization_field_configuration_strategy_t(std::string section)
        : configuration_strategy_t(std::move(section)) {
}

void localization_field_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    auto field_map = field_map_t::get_instance();

    if (prop.count(this->get_section()) == 0) { return; }

    auto& section = prop.get_child(this->get_section());

    auto field_width = section.get_optional<float>("field_width");
    auto field_height = section.get_optional<float>("field_height");
    auto gate_height = section.get_optional<float>("gate_height");
    auto penalty_width = section.get_optional<float>("penalty_width");
    auto penalty_height = section.get_optional<float>("penalty_height");
    auto spawn_x = section.get_optional<float>("spawn_x");
    auto spawn_y = section.get_optional<float>("spawn_y");
    auto spawn_theta = section.get_optional<float>("spawn_theta");
    auto start_x = section.get_optional<float>("start_x");
    auto start_y = section.get_optional<float>("start_y");
    auto start_theta = section.get_optional<float>("start_theta");

    if (field_width) { field_map->set_field_width(field_width.get()); }
    if (field_height) { field_map->set_field_height(field_height.get()); }
    if (gate_height) { field_map->set_gate_height(gate_height.get()); }
    if (penalty_width) { field_map->set_penalty_width(penalty_width.get()); }
    if (penalty_height) { field_map->set_penalty_height(penalty_height.get()); }
    if (spawn_x && spawn_y && spawn_theta) field_map->set_spawn_pose({spawn_x.get(), spawn_y.get(), radians(spawn_theta.get())});
    if (start_x && start_y && start_theta) field_map->set_start_pose({start_x.get(), start_y.get(), radians(start_theta.get())});
}

void localization_field_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    //TODO: :^)
}

