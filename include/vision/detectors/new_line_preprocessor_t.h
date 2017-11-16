/// \autor arssivka
/// \date 11/2/17

#pragma once


#include "preprocessor_t.h"

namespace drwn {
    class new_line_preprocessor_t : public preprocessor_t {
    public:
        cv::Mat preprocess(const cv::Mat& img) const override;

        const cv::Scalar& get_threshold_gabor_bgr_min() const noexcept;

        void set_threshold_gabor_bgr_min(const cv::Scalar& threshold_bgr_min);

        const cv::Scalar& get_threshold_gabor_bgr_max() const noexcept;

        void set_threshold_gabor_bgr_max(const cv::Scalar& threshold_bgr_max);

        const cv::Scalar& get_threshold_gabor_hsv_min() const noexcept;

        void set_threshold_color_hsv_min(const cv::Scalar& threshold_hsv_min);

        const cv::Scalar& get_threshold_color_hsv_max() const noexcept;

        void set_threshold_hsv_max(const cv::Scalar& threshold_hsv_max);

    private:
        cv::Scalar m_threshold_gabor_bgr_min{0, 0, 130}; // TODO Check the order
        cv::Scalar m_threshold_gabor_bgr_max{180, 40, 255};
        cv::Scalar m_threshold_color_hsv_min{0, 0, 130};
        cv::Scalar m_threshold_color_hsv_max{180, 40, 255};
    };
}



