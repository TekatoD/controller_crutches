//
// Created by nikitas on 4/1/16.
//

#pragma once

namespace drwn {
    class ball_detector_t {
    public:
        cv::Rect detect_white_ball(const cv::Mat& image, const std::vector<cv::Vec4i>& lines);

        cv::Rect detect_coloured_ball(const cv::Mat& image);

        bool IsWhite() const noexcept;

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

        cv::Scalar m_min_gabor_bgr_color;
        cv::Scalar m_max_gabor_bgr_color;

        cv::Scalar m_min_bgr_color;
        cv::Scalar m_max_bgr_color;

        int m_median_blur_size;
        bool m_debug{false};
    };

}
