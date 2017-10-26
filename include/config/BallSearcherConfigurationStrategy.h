/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#pragma once

#include <BallSearcher.h>
#include "ConfigurationStrategy.h"


namespace Robot {
    class BallSearcherConfigurationStrategy
            : public ConfigurationStrategy {
    public:

        BallSearcherConfigurationStrategy();

        explicit BallSearcherConfigurationStrategy(BallSearcher* ballSearcher);

        void ReadConfig(const boost::property_tree::ptree& prop) override;

        void WriteConfig(boost::property_tree::ptree& prop) const override;

    private:
        BallSearcher* m_ball_searcher;
    };
}