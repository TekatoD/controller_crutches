/// \autor arssivka
/// \date 11/2/17

#pragma once


#include <opencv2/core/mat.hpp>

namespace drwn {
    class white_ball_detector_t {
    public:
        cv::Rect detect(const cv::Mat& prep_img, const std::vector<cv::Vec4i>& lines) const;

        int get_area_top() const;

        void set_area_top(int area_top);

        double get_area_low() const;

        void set_area_low(double area_low);

    private:
        int m_area_top{5000};
        double m_area_low{18.0};
    };
}



