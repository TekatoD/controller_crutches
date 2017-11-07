/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/walking_t.h>
#include <motion/motion_manager_t.h>
#include "config/strategies/ball_tracker_configuration_strategy_t.h"

using namespace drwn;

ball_tracker_configuration_strategy_t::ball_tracker_configuration_strategy_t(ball_tracker_t* ball_tracker, std::string section)
        : configuration_strategy_t(std::move(section)), m_ball_tracker(ball_tracker) { }

void ball_tracker_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (m_ball_tracker != nullptr) {
        if (prop.count(get_section()) == 0) return; // Section doesn't exist

        auto& tracker_section = prop.get_child(this->get_section());
        auto no_ball_max_count = tracker_section.get_optional<int>("no_ball_max_count");

        if (no_ball_max_count) m_ball_tracker->set_no_ball_max_count(no_ball_max_count.get());
    }
    else {
        throw std::runtime_error("Ball Tracker configuration load fail: BallTracker nullptr");
    }
}

void ball_tracker_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (m_ball_tracker != nullptr) {
        if (prop.count(get_section()) == 0) prop.add_child(get_section(), {});

        auto& tracker_section = prop.get_child(this->get_section());
        tracker_section.put("tilt_phase_step", m_ball_tracker->get_no_ball_max_count());
    }
    else {
        throw std::runtime_error("Ball Tracker configuration write fail: BallTracker nullptr");
    }
}