//
// Created by pav on 25/01/2017.
//

#pragma once

#include <opencv2/core/mat.hpp>

namespace drwn {
    class field_detector_t {
    public:
        cv::Mat detect(const cv::Mat& preproc_image) const;
    };
}
