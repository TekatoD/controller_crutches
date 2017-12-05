/**
 *  @autor arssivka
 *  @date 12/4/17
 */

#include <motion/modules/kicking_t.h>
#include "config/strategies/kicking_configuration_strategy_t.h"

using namespace drwn;


kicking_configuration_strategy_t::kicking_configuration_strategy_t(std::string section)
        : configuration_strategy_t(std::move(section)) {
    // Do nothing
}

void kicking_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    auto kicking = kicking_t::get_instance();
    if (prop.count(this->get_section()) == 0) { return; }
    auto& section = prop.get_child(this->get_section());

    auto shifting_body_duration = section.get_optional<float>("shifting_body_duration");
    if (shifting_body_duration) kicking->set_shifting_body_duration(shifting_body_duration.get());
    auto kicking_duration = section.get_optional<float>("kicking_duration");
    if (kicking_duration) kicking->set_kicking_duration(kicking_duration.get());
    auto restoring_duration = section.get_optional<float>("restoring_duration");
    if (restoring_duration) kicking->set_restoring_duration(restoring_duration.get());
    auto body_init_x_offset = section.get_optional<float>("body_init_x_offset");
    if (body_init_x_offset) kicking->set_body_init_x_offset(body_init_x_offset.get());
    auto body_init_y_offset = section.get_optional<float>("body_init_y_offset");
    if (body_init_y_offset) kicking->set_body_init_y_offset(body_init_y_offset.get());
    auto body_init_z_offset = section.get_optional<float>("body_init_z_offset");
    if (body_init_z_offset) kicking->set_body_init_z_offset(body_init_z_offset.get());
    auto body_init_pitch_offset = section.get_optional<float>("body_init_pitch_offset");
    if (body_init_pitch_offset) kicking->set_body_init_pitch_offset(body_init_pitch_offset.get());
    auto legs_y_offset = section.get_optional<float>("legs_y_offset");
    if (legs_y_offset) kicking->set_legs_y_offset(legs_y_offset.get());
    auto body_x_offset = section.get_optional<float>("body_x_offset");
    if (body_x_offset) kicking->set_body_x_offset(body_x_offset.get());
    auto body_y_offset = section.get_optional<float>("body_y_offset");
    if (body_y_offset) kicking->set_body_y_offset(body_y_offset.get());
    auto body_z_offset = section.get_optional<float>("body_z_offset");
    if (body_z_offset) kicking->set_body_z_offset(body_z_offset.get());
    auto arm_swing_amplitude = section.get_optional<float>("arm_swing_amplitude");
    if (arm_swing_amplitude) kicking->set_arm_swing_amplitude(arm_swing_amplitude.get());
    auto arm_spread_offset = section.get_optional<float>("arm_spread_offset");
    if (arm_spread_offset) kicking->set_arm_spread_offset(arm_spread_offset.get());
    auto elbow_offset = section.get_optional<float>("elbow_offset");
    if (elbow_offset) kicking->set_elbow_offset(elbow_offset.get());
    auto balance_roll_gain = section.get_optional<float>("balance_roll_gain");
    if (balance_roll_gain) kicking->set_balance_roll_gain(balance_roll_gain.get());
    auto balance_pitch_gain = section.get_optional<float>("balance_pitch_gain");
    if (balance_pitch_gain) kicking->set_balance_pitch_gain(balance_pitch_gain.get());
    auto balance_enabled = section.get_optional<bool>("balance_enabled");
    if (balance_enabled) kicking->set_balance_enabled(balance_enabled.get());
}

void kicking_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    throw std::runtime_error("Save kicking configuratin isn't impemented");
}
