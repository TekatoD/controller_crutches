/// \autor arssivka
/// \date 11/2/17

#pragma once


#include <opencv2/core/mat.hpp>
#include <cv.hpp>
#include "ball_preprocessor_t.h"
#include "coloured_ball_detector_t.h"

namespace drwn {
    class white_ball_detector_t {
    public:
        cv::Rect detect(const cv::Mat& prep_img, const cv::Mat& src_img, const std::vector<cv::Vec4i>& lines) const;

        int get_area_top() const;

        void set_area_top(int area_top);

        double get_area_low() const;

        void set_area_low(double area_low);

        int get_detector_type() const;

        void set_detector_type(int detector_type);

        void set_cascade_config(std::string path);

        const std::string& get_cascade_config() const;

        const cv::Scalar& get_threshold_gabor_bgr_min() const;

        void set_threshold_gabor_bgr_min(const cv::Scalar& threshold_gabor_bgr_min);

        const cv::Scalar& get_threshold_gabor_bgr_max() const;

        void set_threshold_gabor_bgr_max(const cv::Scalar& threshold_gabor_bgr_max);

        const cv::Scalar& get_threshold_color_bgr_min() const;

        void set_threshold_color_bgr_min(const cv::Scalar& threshold_color_bgr_min);

        const cv::Scalar& get_threshold_color_bgr_max() const;

        void set_threshold_color_bgr_max(const cv::Scalar& threshold_color_bgr_max);

        int get_median_blur_size() const;

        void set_median_blur_size(int median_blur_size);

    private:
        int m_area_top{5000};
        double m_area_low{18.0};
        int m_detector_type{1};// 0 - white, 1 - haar, 2 - colored
        std::string m_path_to_cascade_config{"res/bottom_cascade.xml"};
        mutable cv::CascadeClassifier m_ball_cascade{"res/bottom_cascade.xml"}; //TODO: May be detect shouldn't be const
        ball_preprocessor_t m_ball_preprocessor;
        coloured_ball_detector_t m_coloured_ball_detector;

    };
}



