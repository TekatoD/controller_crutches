//
// Created by pav on 25/01/2017.
//

#include <cv.hpp>
#include "vision/detectors/field_detector_t.h"

using namespace drwn;

cv::Mat field_detector_t::detect(const cv::Mat& preprocImage) const {
  cv::Rect ans;
  cv::Mat temp_image=cv::Mat::zeros(preprocImage.rows,preprocImage.cols,CV_8UC3);
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(preprocImage, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
  if (!contours.empty()) {
    double maxArea = 0;
    int maxAreaIdx = 0;

    for (int i = 0; i < contours.size(); i++) {
      const double area = cv::contourArea(contours[i]);
      if (area > maxArea) maxArea = area, maxAreaIdx = i;
    }
    for(size_t i = 1; i < contours[maxAreaIdx].size(); ++i){
      cv::line(temp_image, contours[maxAreaIdx][i-1], contours[maxAreaIdx][i], 255);
    }
    cv::line(temp_image, contours[maxAreaIdx][0], contours[maxAreaIdx][contours[maxAreaIdx].size()-1], 255);
    cv::Scalar color(1,1,1);
    cv::drawContours( temp_image, contours, maxAreaIdx, color, CV_FILLED, 8 );

    return temp_image;
  }

  return cv::Mat::zeros(preprocImage.rows,preprocImage.cols,CV_8U);
}
