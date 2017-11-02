/// \autor arssivka
/// \date 11/2/17

#include <cv.hpp>
#include "vision/detectors/old_line_preprocessor_t.h"

using namespace drwn;

cv::Mat old_line_preprocessor_t::preprocess(const cv::Mat& img) const {
    cv::Mat hsv_img, gray_img, bgr_img, buffer;

    cv::cvtColor(img, bgr_img, CV_YUV2BGR);
    cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);
    cv::cvtColor(bgr_img, hsv_img, CV_BGR2HSV);

    const cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10.0, 4.1, M_PI, 0.0);
    cv::Mat filtred_image, thresh_filt_image;
    cv::filter2D(bgr_img, filtred_image, -1, kernel);
    cv::cvtColor(filtred_image, buffer, CV_BGR2Lab);
    cv::inRange(buffer, m_threshold_gabor_bgr_min, m_threshold_gabor_bgr_max, thresh_filt_image);
    cv::adaptiveThreshold(gray_img, gray_img, 255,
                          CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,
                          m_kernel_size, 0.0);

    cv::Mat colorImg = gray_img.clone();

    for (int r = 0; r < colorImg.rows; r++) {
        for (int c = 0; c < colorImg.cols; c++) {
            if (thresh_filt_image.at<uchar>(r, c) != 0) {
                cv::floodFill(colorImg, cv::Point(c, r), 0);
                cv::floodFill(thresh_filt_image, cv::Point(c, r), 0);
            }
        }
    }

    cv::Mat result = gray_img - colorImg;
    return result;
}

const cv::Scalar& old_line_preprocessor_t::get_threshold_gabor_bgr_min() const noexcept {
    return m_threshold_gabor_bgr_min;
}

void old_line_preprocessor_t::set_threshold_gabor_bgr_min(const cv::Scalar& threshold_bgr_min) {
    m_threshold_gabor_bgr_min = threshold_bgr_min;
}

const cv::Scalar& old_line_preprocessor_t::get_threshold_gabor_bgr_max() const noexcept {
    return m_threshold_gabor_bgr_max;
}

void old_line_preprocessor_t::set_threshold_gabor_bgr_max(const cv::Scalar& threshold_bgr_max) {
    m_threshold_gabor_bgr_max = threshold_bgr_max;
}

int old_line_preprocessor_t::get_kernel_size() const noexcept {
    return m_kernel_size;
}

void old_line_preprocessor_t::set_kernel_size(int kernel_size) {
    m_kernel_size = kernel_size;
}
