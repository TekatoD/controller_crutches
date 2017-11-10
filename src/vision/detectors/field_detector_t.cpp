//
// Created by pav on 25/01/2017.
//

#include <cv.hpp>
#include "vision/detectors/field_detector_t.h"

using namespace drwn;

cv::Mat field_detector_t::detect(const cv::Mat& preproc_image) const {
    cv::Mat result = cv::Mat::zeros(preproc_image.rows, preproc_image.cols, CV_8UC1);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(preproc_image, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        double max_area = 0;
        int max_area_idx = 0;

        for (int i = 0; i < contours.size(); i++) {
            const double area = cv::contourArea(contours[i]);
            if (area > max_area) max_area = area, max_area_idx = i;
        }
        for (size_t i = 1; i < contours[max_area_idx].size(); ++i) {
            cv::line(result, contours[max_area_idx][i - 1], contours[max_area_idx][i], 255);
        }
        cv::line(result, contours[max_area_idx][0], contours[max_area_idx][contours[max_area_idx].size() - 1], 255);
        cv::Scalar color(1, 1, 1);
        cv::drawContours(result, contours, max_area_idx, color, CV_FILLED, 8);
    }
    return result;
}
