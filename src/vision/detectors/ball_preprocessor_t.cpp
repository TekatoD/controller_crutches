/// \autor arssivka
/// \date 11/2/17

#include <cv.hpp>
#include "vision/detectors/ball_preprocessor_t.h"

using namespace drwn;

cv::Mat ball_preprocessor_t::preprocess(const cv::Mat& img) const {
    cv::Mat prep_img;
    cv::cvtColor(img, prep_img, CV_YUV2BGR); // TODO Optimize it
    cv::Mat median_blur_frame;
    cv::medianBlur(prep_img, median_blur_frame, m_median_blur_size);
    cv::Mat afterGaborRange, gaborImage;
    cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10, 7.8, 9.4, 0);
    cv::filter2D(median_blur_frame, gaborImage, -1, kernel);

    cv::Mat gaborInRange;
    cv::inRange(gaborImage, m_threshold_gabor_bgr_min, m_threshold_gabor_bgr_max, gaborInRange);

    cv::morphologyEx(gaborInRange, afterGaborRange, CV_MOP_DILATE, cv::Mat::ones(3, 3, CV_8UC1));

    cv::inRange(median_blur_frame, m_threshold_color_bgr_min, m_threshold_color_bgr_max, prep_img);
    cv::morphologyEx(prep_img, prep_img, CV_MOP_DILATE, cv::Mat::ones(3, 3, CV_8UC1));

    cv::Mat ball = prep_img.clone();
    for (int r = 0; r < ball.rows; r++) {
        for (int c = 0; c < ball.cols; c++) {
            if (afterGaborRange.at<uchar>(r, c) != 0) {
                cv::floodFill(ball, cv::Point(c, r), 0);
            }
        }
    }
    return prep_img - ball;
}

const cv::Scalar& ball_preprocessor_t::get_threshold_gabor_bgr_min() const {
    return m_threshold_gabor_bgr_min;
}

void ball_preprocessor_t::set_threshold_gabor_bgr_min(const cv::Scalar& threshold_gabor_bgr_min) {
    m_threshold_gabor_bgr_min = threshold_gabor_bgr_min;
}

const cv::Scalar& ball_preprocessor_t::get_threshold_gabor_bgr_max() const {
    return m_threshold_gabor_bgr_max;
}

void ball_preprocessor_t::set_threshold_gabor_bgr_max(const cv::Scalar& threshold_gabor_bgr_max) {
    m_threshold_gabor_bgr_max = threshold_gabor_bgr_max;
}

const cv::Scalar& ball_preprocessor_t::get_threshold_color_bgr_min() const {
    return m_threshold_color_bgr_min;
}

void ball_preprocessor_t::set_threshold_color_bgr_min(const cv::Scalar& threshold_color_bgr_min) {
    m_threshold_color_bgr_min = threshold_color_bgr_min;
}

const cv::Scalar& ball_preprocessor_t::get_threshold_color_bgr_max() const {
    return m_threshold_color_bgr_max;
}

void ball_preprocessor_t::set_threshold_color_bgr_max(const cv::Scalar& threshold_color_bgr_max) {
    m_threshold_color_bgr_max = threshold_color_bgr_max;
}

int ball_preprocessor_t::get_median_blur_size() const {
    return m_median_blur_size;
}

void ball_preprocessor_t::set_median_blur_size(int median_blur_size) {
    m_median_blur_size = median_blur_size;
}
