/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/walking_t.h>
#include <motion/motion_manager_t.h>
#include "config/strategies/ball_searcher_configuration_strategy_t.h"

using namespace drwn;

ball_searcher_configuration_strategy_t::ball_searcher_configuration_strategy_t(std::string section)
        : configuration_strategy_t(std::move(section)) {}

void ball_searcher_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (prop.count(this->get_section()) == 0) return; // Section doesn't exist

    auto& searcher_section = prop.get_child(this->get_section());
    auto tilt_phase_step = searcher_section.get_optional<float>("tilt_phase_step");
    auto pan_phase_step = searcher_section.get_optional<float>("pan_phase_step");
    auto phase_size = searcher_section.get_optional<float>("phase_size");
    auto turn_step = searcher_section.get_optional<float>("turn_step");
    auto max_turn = searcher_section.get_optional<float>("max_turn");
    auto pan_enabled = searcher_section.get_optional<bool>("pan_enabled");

    if (tilt_phase_step) ball_searcher_t::get_instance()->set_tilt_phase_step(tilt_phase_step.get());
    if (pan_phase_step) ball_searcher_t::get_instance()->set_pan_phase_step(pan_phase_step.get());
    if (phase_size) ball_searcher_t::get_instance()->set_phase_size(phase_size.get());
    if (turn_step) ball_searcher_t::get_instance()->set_turn_step(turn_step.get());
    if (max_turn) ball_searcher_t::get_instance()->set_max_turn(max_turn.get());
    if (pan_enabled) ball_searcher_t::get_instance()->enable_pan(pan_enabled.get());
}

void ball_searcher_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (prop.count(this->get_section()) == 0) prop.add_child(get_section(), {});

    auto& searcher_section = prop.get_child(this->get_section());
    searcher_section.put("tilt_phase_step", ball_searcher_t::get_instance()->get_tilt_phase_step());
    searcher_section.put("pan_phase_step", ball_searcher_t::get_instance()->get_pan_phase_step());
    searcher_section.put("phase_size", ball_searcher_t::get_instance()->get_phase_size());
    searcher_section.put("turn_step", ball_searcher_t::get_instance()->get_turn_step());
    searcher_section.put("max_turn", ball_searcher_t::get_instance()->get_max_turn());
    searcher_section.put("pan_enabled", ball_searcher_t::get_instance()->is_pan_enabled());

}