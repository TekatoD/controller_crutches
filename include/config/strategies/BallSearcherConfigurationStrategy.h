/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#pragma once

#include <BallSearcher.h>
#include "config/ConfigurationStrategy.h"


namespace drwn {
    class BallSearcherConfigurationStrategy
            : public ConfigurationStrategy {
    public:

        static constexpr char DEFAULT_SECTION[] = "Ball Searcher";

        explicit BallSearcherConfigurationStrategy(BallSearcher* ballSearcher = nullptr, std::string section = DEFAULT_SECTION);

        void ReadConfig(const boost::property_tree::ptree& prop) override;

        void WriteConfig(boost::property_tree::ptree& prop) const override;

    private:
        BallSearcher* m_ball_searcher{nullptr};
    };
}
