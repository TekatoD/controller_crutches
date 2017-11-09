/// \autor arssivka
/// \date 11/2/17

#include <cv.hpp>
#include "vision/detectors/new_line_preprocessor_t.h"

using namespace drwn;

cv::Mat new_line_preprocessor_t::preprocess(const cv::Mat& img) const {
    constexpr auto kernel_theta = 4.1;
    constexpr auto kernel_sigma = 10.0;
    constexpr auto kernel_lambda = M_PI;
    constexpr auto kernel_gamma = 0.0;

    cv::Mat hsv_img, gray_img, bgr_img, tmp, yuv_test;
    cv::cvtColor(img, yuv_test, CV_BGR2YUV);
    cv::cvtColor(yuv_test, bgr_img, CV_YUV2BGR);
    cv::GaussianBlur(bgr_img, bgr_img, cv::Size(5, 5), 1.5, 1.5);
    cv::cvtColor(bgr_img, hsv_img, CV_BGR2HSV);

    const cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), kernel_sigma, kernel_theta,
                                              kernel_lambda, kernel_gamma);
    cv::Mat filtred_img, result;
    cv::filter2D(bgr_img, filtred_img, -1, kernel);
    cv::cvtColor(filtred_img, tmp, CV_BGR2Lab);
    cv::inRange(tmp, m_threshold_gabor_bgr_min, m_threshold_gabor_bgr_max, result);
    cv::Mat mask;
    cv::inRange(hsv_img, m_threshold_color_hsv_min, m_threshold_color_hsv_max, mask);
    mask.copyTo(result, mask);
    return result;
}

const cv::Scalar& new_line_preprocessor_t::get_threshold_gabor_bgr_min() const noexcept {
    return m_threshold_gabor_bgr_min;
}

void new_line_preprocessor_t::set_threshold_gabor_bgr_min(const cv::Scalar& threshold_bgr_min) {
    m_threshold_gabor_bgr_min = threshold_bgr_min;
}

const cv::Scalar& new_line_preprocessor_t::get_threshold_gabor_bgr_max() const noexcept {
    return m_threshold_gabor_bgr_max;
}

void new_line_preprocessor_t::set_threshold_gabor_bgr_max(const cv::Scalar& threshold_bgr_max) {
    m_threshold_gabor_bgr_max = threshold_bgr_max;
}

const cv::Scalar& new_line_preprocessor_t::get_threshold_gabor_hsv_min() const noexcept {
    return m_threshold_color_hsv_min;
}

void new_line_preprocessor_t::set_threshold_color_hsv_min(const cv::Scalar& threshold_hsv_min) {
    m_threshold_color_hsv_min = threshold_hsv_min;
}

const cv::Scalar& new_line_preprocessor_t::get_threshold_color_hsv_max() const noexcept {
    return m_threshold_color_hsv_max;
}

void new_line_preprocessor_t::set_threshold_hsv_max(const cv::Scalar& threshold_hsv_max) {
    m_threshold_color_hsv_max = threshold_hsv_max;
}
