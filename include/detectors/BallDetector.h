//
// Created by nikitas on 4/1/16.
//

#ifndef NAOMECH_BALLDETECTOR_H
#define NAOMECH_BALLDETECTOR_H

#include "minIni.h"
#include "BaseDetector.h"

namespace ant {

  class BallDetector : public BaseDetector {
  public:
    BallDetector();

    cv::Mat preproccess(const cv::Mat &image);

    cv::Rect detect(const cv::Mat &image, const std::vector<cv::Vec4i> &m_lines);

    cv::Rect detect_old(const cv::Mat &image);

    bool is_white();

    struct configuration {
      bool white_ball;
      struct {
        uchar min_1, max_1;
        uchar min_2, max_2;
        uchar min_3, max_3;
      } ColorThresh;

      struct {
        uchar min_1, max_1;
        uchar min_2, max_2;
        uchar min_3, max_3;
      } GaborThresh;

      int median_blur_size;
    };

    void load(minIni* ini);

    configuration m_conf;

  private:

  };

}

#endif //NAOMECH_BALLDETECTOR_H
