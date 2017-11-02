/// \autor arssivka
/// \date 11/2/17

#pragma once


#include "preprocessor_t.h"

namespace drwn {
    class ball_preprocessor_t : public preprocessor_t {
    public:
        cv::Mat preprocess(const cv::Mat& img) const override;

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
        cv::Scalar m_threshold_gabor_bgr_min;
        cv::Scalar m_threshold_gabor_bgr_max;
        cv::Scalar m_threshold_color_bgr_min;
        cv::Scalar m_threshold_color_bgr_max;

        int m_median_blur_size;
    };
}



