/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#pragma once

#include <BallTracker.h>
#include "config/ConfigurationStrategy.h"

namespace Robot {
    class BallTrackerConfigurationStrategy
            : public ConfigurationStrategy {
    public:

        static constexpr char DEFAULT_SECTION[] = "Ball Tracker";

        explicit BallTrackerConfigurationStrategy(BallTracker* ballTracker = nullptr, std::string section = DEFAULT_SECTION);

        void ReadConfig(const boost::property_tree::ptree& prop) override;

        void WriteConfig(boost::property_tree::ptree& prop) const override;

    private:
        BallTracker* m_ball_tracker{nullptr};
    };
}