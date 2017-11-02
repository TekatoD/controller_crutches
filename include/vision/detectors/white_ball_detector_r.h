/// \autor arssivka
/// \date 11/2/17

#pragma once


#include <opencv2/core/mat.hpp>

namespace drwn {
    class white_ball_detector_r {
    public:
        cv::Rect detect(const cv::Mat& prep_img, const std::vector<cv::Vec4i>& lines) const;
    };
}



