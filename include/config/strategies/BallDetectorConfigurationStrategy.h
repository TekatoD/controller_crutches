/**
 *  @autor tekatod
 *  @date 10/31/17
 */
#pragma once

#include <vision/detectors/ball_detector_t.h>
#include "config/ConfigurationStrategy.h"

namespace drwn {
    class BallDetectorConfigurationStrategy
            : public ConfigurationStrategy {
    public:

        static constexpr char DEFAULT_SECTION[] = "BallDetector";

        explicit BallDetectorConfigurationStrategy(ball_detector_t* ballDetector = nullptr, std::string section = DEFAULT_SECTION);

        void ReadConfig(const boost::property_tree::ptree& prop) override;

        void WriteConfig(boost::property_tree::ptree& prop) const override;

    private:
        ball_detector_t* m_ball_detector{nullptr};
    };
}