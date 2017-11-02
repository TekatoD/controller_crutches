/**
 *  @autor tekatod
 *  @date 10/31/17
 */
#pragma once

#include <vision/detectors/BallDetector.h>
#include "config/configuration_strategy_t.h"

namespace drwn {
    class ball_detector_configuration_strategy_t
            : public configuration_strategy_t {
    public:

        static constexpr char DEFAULT_SECTION[] = "BallDetector";

        explicit ball_detector_configuration_strategy_t(BallDetector* ballDetector = nullptr, std::string section = DEFAULT_SECTION);

        void read_config(const boost::property_tree::ptree& prop) override;

        void write_config(boost::property_tree::ptree& prop) const override;

    private:
        BallDetector* m_ball_detector{nullptr};
    };
}