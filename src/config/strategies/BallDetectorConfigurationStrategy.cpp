/**
 *  @autor tekatod
 *  @date 10/31/17
 */

#include "config/strategies/BallDetectorConfigurationStrategy.h"

drwn::BallDetectorConfigurationStrategy::BallDetectorConfigurationStrategy(drwn::ball_detector_t* ballDetector,
                                                                            std::string section) :
        ConfigurationStrategy(std::move(section)), m_ball_detector(ballDetector) {

}

void drwn::BallDetectorConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    if (m_ball_detector != nullptr) {
        if (prop.count(GetSection()) == 0) return; // Section doesn't exist
        auto& searcher_section = prop.get_child(this->GetSection());
        for (size_t i = 0; i < 3; ++i) {
            auto max_param = searcher_section.get_optional<uchar>("ColorThresh.max_" + std::to_string(i + 1));
            auto min_param = searcher_section.get_optional<uchar>("ColorThresh.min_" + std::to_string(i + 1));
            if(max_param) {
                m_ball_detector->SetMaxColorThresh(i, max_param.get());
            }
            if(min_param) {
                m_ball_detector->SetMinColorThresh(i, min_param.get());
            }
        }
        for (size_t i = 0; i < 3; ++i) {
            auto max_param = searcher_section.get_optional<uchar>("GaborThresh.max_" + std::to_string(i + 1));
            auto min_param = searcher_section.get_optional<uchar>("GaborThresh.min_" + std::to_string(i + 1));
            if(max_param) {
                m_ball_detector->SetMaxGaborThresh(i, max_param.get());
            }
            if(min_param) {
                m_ball_detector->SetMinGaborThresh(i, min_param.get());
            }
        }
        auto median_size = searcher_section.get_optional<int>("median_blur_size");
        if(median_size) {
            m_ball_detector->SetMedianBlurSize(median_size.get());
        }
    }
}

void drwn::BallDetectorConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    if (m_ball_detector != nullptr) {
        if (prop.count(GetSection()) == 0) prop.add_child(GetSection(), {});

        auto& searcher_section = prop.get_child(this->GetSection());
        for (size_t i = 0; i < 3; ++i) {
            searcher_section.put("GaborThresh.max_" + std::to_string(i + 1), m_ball_detector->GetMaxGaborThresh(i));
            searcher_section.put("GaborThresh.min_" + std::to_string(i + 1), m_ball_detector->GetMinGaborThresh(i));
        }
        for (size_t i = 0; i < 3; ++i) {
            searcher_section.put("ColorThresh.max_" + std::to_string(i + 1), m_ball_detector->GetMaxColorThresh(i));
            searcher_section.put("ColorThresh.min_" + std::to_string(i + 1), m_ball_detector->GetMinColorThresh(i));
        }
        searcher_section.put("median_blur_size", m_ball_detector->GetMedianBlurSize());
    }
}
