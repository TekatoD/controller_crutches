/// \autor arssivka
/// \date 11/2/17

#include <cv.hpp>
#include "vision/detectors/coloured_ball_detector_t.h"

cv::Rect drwn::coloured_ball_detector_t::detect(const cv::Mat& preproc_img) const {
    cv::Rect result;
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(preproc_img, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        double max_area = 0;
        int max_area_idx = 0;

        for (int i = 0; i < contours.size(); i++) {
            const double area = cv::contourArea(contours[i]);
            if (area > max_area) max_area = area, max_area_idx = i;
        }

        result = cv::boundingRect(contours[max_area_idx]);
    }
    return result;
}
