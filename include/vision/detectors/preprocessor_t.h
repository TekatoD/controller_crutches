/// \autor arssivka
/// \date 11/2/17

#pragma once


#include <opencv2/core/mat.hpp>

namespace drwn {
    class preprocessor_t {
    public:
        virtual cv::Mat preprocess(const cv::Mat& img) const = 0;

    };
}



