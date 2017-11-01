//
// Created by nikitas on 26.03.16.
//

#pragma once

#include "BaseDetector.h"

namespace drwn {
    class LineDetector : public BaseDetector {
    public:
        LineDetector();

        cv::Mat Preproccess(const cv::Mat& image);

        cv::Mat PreproccessOld(const cv::Mat& image);

        std::vector<cv::Vec4i> Detect(const cv::Mat& preproc_image);

        std::vector<cv::Vec4i> DetectOld(const cv::Mat& preproc_image);

        struct configuration {
            struct HoughLines {
                double rho;
                double theta;
                double min_line_length;
                double max_line_gap;
                int threshold;
            } HoughLines;

            struct Preproc {
                int min_thresh;
                int kernel_size;
                int kernel_size_2;
                struct {
                    uchar min_1, max_1;
                    uchar min_2, max_2;
                    uchar min_3, max_3;
                } ColorThresh, ColorThresh2;
            } Preproc, Preproc_new;

            struct LineEqualPredicate {
                float angle_eps;
                int error_px;

                bool operator()(const cv::Vec4i &line1, const cv::Vec4i &line2);
            } LineEqualPredicate;
        };

        void load(const boost::property_tree::ptree &line_config);

        boost::property_tree::ptree get_params();

    private:
        void GetSkeleton(const cv::Mat& img, cv::Mat& result);
        void GetSimpleSkeleton(const cv::Mat& img, cv::Mat& result);
        void ZhangSuen(const cv::Mat& img, cv::Mat& result);

        void JoinLines(std::vector<cv::Vec4i>& lines);

    private:
        configuration m_conf;
    };


}
