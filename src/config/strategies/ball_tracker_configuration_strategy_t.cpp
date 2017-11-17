/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/walking_t.h>
#include <motion/motion_manager_t.h>
#include "config/strategies/ball_tracker_configuration_strategy_t.h"

using namespace drwn;

ball_tracker_configuration_strategy_t::ball_tracker_configuration_strategy_t(std::string section)
        : configuration_strategy_t(std::move(section)) {}

void ball_tracker_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (prop.count(get_section()) == 0) return; // Section doesn't exist

    auto& tracker_section = prop.get_child(this->get_section());
    auto no_ball_max_count = tracker_section.get_optional<int>("no_ball_max_count");

    if (no_ball_max_count) ball_tracker_t::get_instance()->set_no_ball_max_count(no_ball_max_count.get());
}

void ball_tracker_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (prop.count(get_section()) == 0) prop.add_child(get_section(), {});

    auto& tracker_section = prop.get_child(this->get_section());
    tracker_section.put("tilt_phase_step", ball_tracker_t::get_instance()->get_no_ball_max_count());
}