/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#pragma once

#include <BallTracker.h>
#include "ConfigurationStrategy.h"

namespace Robot {
    class BallTrackerConfigurationStrategy
            : public ConfigurationStrategy {
    public:
        BallTrackerConfigurationStrategy();

        explicit BallTrackerConfigurationStrategy(BallTracker* ballTracker);

        void ReadConfig(const boost::property_tree::ptree& prop) override;

        void WriteConfig(boost::property_tree::ptree& prop) const override;

    private:
        BallTracker* m_ball_tracker;

    };
}