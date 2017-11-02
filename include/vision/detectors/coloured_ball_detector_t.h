/// \autor arssivka
/// \date 11/2/17

#pragma once


#include <opencv2/core/types.hpp>

namespace drwn {
    class coloured_ball_detector_t {
    public:
        cv::Rect detect(const cv::Mat& preproc_img) const;
    };
}



