/// \autor arssivka
/// \date 11/2/17

#pragma once


#include "preprocessor_t.h"

namespace drwn {
    class old_line_preprocessor_t : public preprocessor_t {
    public:
        cv::Mat preprocess(const cv::Mat& img) const override;

        const cv::Scalar& get_threshold_gabor_bgr_min() const noexcept;

        void set_threshold_gabor_bgr_min(const cv::Scalar& threshold_bgr_min);

        const cv::Scalar& get_threshold_gabor_bgr_max() const noexcept;

        void set_threshold_gabor_bgr_max(const cv::Scalar& threshold_bgr_max);

        int get_kernel_size() const noexcept;

        void set_kernel_size(int kernel_size);

    private:
        cv::Scalar m_threshold_gabor_bgr_min{160, 90, 0}; // TODO Check the order
        cv::Scalar m_threshold_gabor_bgr_max{255, 238, 142}; // TODO Check the order
        int m_kernel_size{71};
    };
}



