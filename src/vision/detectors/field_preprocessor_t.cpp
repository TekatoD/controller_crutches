/// \autor arssivka
/// \date 11/2/17

#include <cv.hpp>
#include "vision/detectors/field_preprocessor_t.h"

cv::Mat drwn::field_preprocessor_t::preprocess(const cv::Mat& img) const {
    cv::Mat preproc_img, hsv_img;
    img.copyTo(preproc_img);
    cv::cvtColor(preproc_img,hsv_img,cv::COLOR_BGR2HSV);
    cv::Mat medianBlurFrame;
    cv::medianBlur(preproc_img, medianBlurFrame, 3);
    cv::Mat afterGaborRange, gaborImage;
    cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10, 7.8, 9.4, 0);
    cv::filter2D(medianBlurFrame, gaborImage, -1, kernel);

    cv::Mat gabor_in_range;

    cv::inRange(gaborImage, m_threshold_gabor_bgr_min, m_threshold_gabor_bgr_max, gabor_in_range);
    cv::morphologyEx(gabor_in_range, afterGaborRange, CV_MOP_DILATE, cv::Mat::ones(3, 3, CV_8UC1));

    cv::inRange(hsv_img, m_threshold_color_hsv_min, m_threshold_color_hsv_max, preproc_img);
    cv::morphologyEx(preproc_img, preproc_img, CV_MOP_DILATE, cv::Mat::ones(7, 7, CV_8UC1));

    cv::Mat field = preproc_img.clone();
    for (int r = 0; r < field.rows; r++) {
        for (int c = 0; c < field.cols; c++) {
            if (afterGaborRange.at<uchar>(r, c) != 0) {
                cv::floodFill(field, cv::Point(c, r), 0);
            }
        }
    }

    return preproc_img + field;
}

const cv::Scalar& drwn::field_preprocessor_t::get_threshold_gabor_bgr_min() const {
    return m_threshold_gabor_bgr_min;
}

void drwn::field_preprocessor_t::set_threshold_gabor_bgr_min(const cv::Scalar& threshold_gabor_bgr_min) {
    m_threshold_gabor_bgr_min = threshold_gabor_bgr_min;
}

const cv::Scalar& drwn::field_preprocessor_t::get_threshold_gabor_bgr_max() const {
    return m_threshold_gabor_bgr_max;
}

void drwn::field_preprocessor_t::set_threshold_gabor_bgr_max(const cv::Scalar& threshold_gabor_bgr_max) {
    m_threshold_gabor_bgr_max = threshold_gabor_bgr_max;
}

const cv::Scalar& drwn::field_preprocessor_t::get_threshold_color_hsv_min() const {
    return m_threshold_color_hsv_min;
}

void drwn::field_preprocessor_t::set_threshold_color_hsv_min(const cv::Scalar& threshold_color_hsv_min) {
    m_threshold_color_hsv_min = threshold_color_hsv_min;
}

const cv::Scalar& drwn::field_preprocessor_t::get_threshold_color_hsv_max() const {
    return m_threshold_color_hsv_max;
}

void drwn::field_preprocessor_t::set_threshold_color_hsv_max(const cv::Scalar& threshold_color_hsv_max) {
    m_threshold_color_hsv_max = threshold_color_hsv_max;
}
