//
// Created by nikitas on 4/1/16.
//

#pragma once

#include "BaseDetector.h"

namespace drwn {
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

        int GetMedianBlurSize() const noexcept;

        int SetMedianBlurSize(int median_blur_sie);

        bool IsDebugEnabled();

        void EnableDebug(bool debug);

        const cv::Scalar& GetMinGaborColor() const;

        void SetMinGaborColor(const cv::Scalar& min_gabor_color);

        const cv::Scalar& GetMaxGaborColor() const;

        void SetMaxGaborColor(const cv::Scalar& max_gabor_color);

        const cv::Scalar& GetM_min_color() const;

        void SetMinColor(const cv::Scalar& min_color);

        const cv::Scalar& GetMaxColor() const;

        void SetMaxColor(const cv::Scalar& max_color);

    private:
        configuration m_conf;
        cv::Scalar m_min_gabor_color;
        cv::Scalar m_max_gabor_color;

        cv::Scalar m_min_color;
        cv::Scalar m_max_color;

        int m_median_blur_size;
        bool m_debug{false};
    };

}
