/**
 *  @autor arssivka
 *  @date 11/27/17
 */

#include <behavior/go_to_t.h>
#include "config/strategies/go_to_configuration_strategy_t.h"

drwn::go_to_configuration_strategy_t::go_to_configuration_strategy_t(std::string section)
        : configuration_strategy_t(section) {}

void drwn::go_to_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (prop.count(this->get_section()) == 0) return; // Section doesn't exist
    auto& section = prop.get_child(this->get_section());
    auto go_to = go_to_t::get_instance();

    auto max_speed = section.get_optional<float>("max_speed");
    if (max_speed) go_to->set_max_speed(max_speed.get());
    auto fit_speed = section.get_optional<float>("fit_speed");
    if (fit_speed) go_to->set_fit_speed(fit_speed.get());
    auto max_turn = section.get_optional<float>("max_turn");
    if (max_turn) go_to->set_max_turn(max_turn.get());
    auto step_accel = section.get_optional<float>("step_accel");
    if (step_accel) go_to->set_step_accel(step_accel.get());
    auto turn_accel = section.get_optional<float>("turn_accel");
    if (turn_accel) go_to->set_turn_accel(turn_accel.get());
    auto fit_distance = section.get_optional<float>("fit_distance");
    if (fit_distance) go_to->set_fit_distance(fit_distance.get());
    auto distance_var = section.get_optional<float>("distance_var");
    if (distance_var) go_to->set_distance_var(distance_var.get());
    auto angle_var = section.get_optional<float>("angle_var");
    if (angle_var) go_to->set_angle_var(angle_var.get());
}

void drwn::go_to_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    throw std::runtime_error("go_to_t::write_config isn't implemented"); // TODO Do sometginf
}
