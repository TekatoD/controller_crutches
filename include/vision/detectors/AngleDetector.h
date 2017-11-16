//
// Created by pav on 27/12/2016.
//

#pragma once

namespace drwn {
    class AngleDetector : public BaseDetector {
    public:
        AngleDetector();

        std::vector<cv::Vec3d> Detect(cv::Mat& img, const std::vector<cv::Vec4i>& lines);

        void load(const boost::property_tree::ptree &config);

        boost::property_tree::ptree get_params();

    private:
        std::pair<double, double> GetLineParams(const cv::Vec4i& line);

        bool IsOnLine(const cv::Vec4i& line, double x, double y);

    };
}
