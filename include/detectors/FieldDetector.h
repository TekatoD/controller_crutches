//
// Created by pav on 25/01/2017.
//

#ifndef DEMO_FIELDDETECTOR_H
#define DEMO_FIELDDETECTOR_H

#include <minIni.h>
#include <detectors/BaseDetector.h>

namespace ant {
  class FieldDetector : BaseDetector{
    struct {
      uchar min_1, max_1;
      uchar min_2, max_2;
      uchar min_3, max_3;
    } ColorThresh, ColorThresh2;
  public:
      FieldDetector();

      cv::Mat preproccess(const cv::Mat &image);

      cv::Mat detect(const cv::Mat &image);

      void load(minIni* ini);
  };
}

#endif //DEMO_FIELDDETECTOR_H
