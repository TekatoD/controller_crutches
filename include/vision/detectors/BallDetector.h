//
// Created by nikitas on 4/1/16.
//

#pragma once

#include "BaseDetector.h"

namespace Robot {
    class BallDetector : public BaseDetector {
    public:
        struct configuration {
            bool white_ball;
            struct {
                uchar min_1, max_1;
                uchar min_2, max_2;
                uchar min_3, max_3;
            } ColorThresh;

            struct {
                uchar min_1, max_1;
                uchar min_2, max_2;
                uchar min_3, max_3;
            } GaborThresh;

            int median_blur_size;
        };

        BallDetector();

        cv::Mat Preproccess(const cv::Mat& image);

        cv::Rect Detect(const cv::Mat& image, const std::vector<cv::Vec4i>& lines);

        cv::Rect DetectOld(const cv::Mat& image);

        bool IsWhite() const noexcept;

        void load(const boost::property_tree::ptree& ball_config);

        boost::property_tree::ptree get_params();

    private:
        configuration m_conf;

    };

}
