/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/Walking.h>
#include <motion/MotionManager.h>
#include "config/BallTrackerConfigurationStrategy.h"

using namespace Robot;

BallTrackerConfigurationStrategy::BallTrackerConfigurationStrategy(BallTracker* ballTracker, std::string section)
        : ConfigurationStrategy(std::move(section)), m_ball_tracker(ballTracker) { }

void BallTrackerConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    if (m_ball_tracker != nullptr) {
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
        auto& tracker_section = prop.get_child(this->GetSection());
        tracker_section.put("tilt_phase_step", m_ball_tracker->GetNoBallMaxCount());
    }
    else {
        throw std::runtime_error("Ball Tracker configuration write fail: BallTracker nullptr");
    }
}