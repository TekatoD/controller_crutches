/// \autor arssivka
/// \date 11/2/17

#pragma once


#include "preprocessor_t.h"

namespace drwn {
    class field_preprocessor_t : public preprocessor_t {
    public:
        cv::Mat preprocess(const cv::Mat& img) const override;

        const cv::Scalar& get_threshold_gabor_bgr_min() const;

        void set_threshold_gabor_bgr_min(const cv::Scalar& threshold_gabor_bgr_min);

        const cv::Scalar& get_threshold_gabor_bgr_max() const;

        void set_threshold_gabor_bgr_max(const cv::Scalar& threshold_gabor_bgr_max);

        const cv::Scalar& get_threshold_color_hsv_min() const;

        void set_threshold_color_hsv_min(const cv::Scalar& threshold_color_hsv_min);

        const cv::Scalar& get_threshold_color_hsv_max() const;

        void set_threshold_color_hsv_max(const cv::Scalar& threshold_color_hsv_max);

    private:
        cv::Scalar m_threshold_gabor_bgr_min{0, 50, 40};
        cv::Scalar m_threshold_gabor_bgr_max{190, 190, 140};
        cv::Scalar m_threshold_color_hsv_min{40, 60, 40};
        cv::Scalar m_threshold_color_hsv_max{90, 250, 190};
    };
}



