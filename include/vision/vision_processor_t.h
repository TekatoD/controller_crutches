/// \autor arssivka
/// \date 11/3/17

#pragma once


#include <opencv2/core/mat.hpp>

namespace drwn {
    class vision_processor_t {
    public:
        virtual const cv::Rect& detect_ball() = 0;

        virtual const std::vector<cv::Vec4i>& detect_lines() = 0;

        virtual const std::vector<cv::Vec3d>& detect_angles() = 0;

        virtual const cv::Mat& detect_field() = 0;

        virtual void set_frame(cv::Mat frame) = 0;

        virtual void reset() = 0;

        virtual void process() = 0;

        virtual ~vision_processor_t() = default;
    };
}


