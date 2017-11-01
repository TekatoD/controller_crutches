/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/walking_t.h>
#include <motion/motion_manager_t.h>
#include "config/strategies/BallTrackerConfigurationStrategy.h"

using namespace drwn;

BallTrackerConfigurationStrategy::BallTrackerConfigurationStrategy(BallTracker* ballTracker, std::string section)
        : ConfigurationStrategy(std::move(section)), m_ball_tracker(ballTracker) { }

void BallTrackerConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    if (m_ball_tracker != nullptr) {
        if (prop.count(GetSection()) == 0) return; // Section doesn't exist

        auto& tracker_section = prop.get_child(this->GetSection());
        auto no_ball_max_count = tracker_section.get_optional<int>("no_ball_max_count");

        if (no_ball_max_count) m_ball_tracker->SetNoBallMaxCount(no_ball_max_count.get());
    }
    else {
        throw std::runtime_error("Ball Tracker configuration load fail: BallTracker nullptr");
    }
}

void BallTrackerConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    if (m_ball_tracker != nullptr) {
        if (prop.count(GetSection()) == 0) prop.add_child(GetSection(), {});

        auto& tracker_section = prop.get_child(this->GetSection());
        tracker_section.put("tilt_phase_step", m_ball_tracker->GetNoBallMaxCount());
    }
    else {
        throw std::runtime_error("Ball Tracker configuration write fail: BallTracker nullptr");
    }
}