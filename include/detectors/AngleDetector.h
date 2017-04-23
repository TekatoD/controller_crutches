//
// Created by pav on 27/12/2016.
//

#ifndef DEMO_ANGLEDETECOR_H
#define DEMO_ANGLEDETECOR_H

#include "BaseDetector.h"

namespace ant {

    class AngleDetector : public BaseDetector {
    public:
        AngleDetector();

        std::vector<cv::Vec3d> detect(cv::Mat &img, const std::vector<cv::Vec4i> &lines);

        void load(const boost::property_tree::ptree &config);

        boost::property_tree::ptree get_params();

    private:
        std::pair<double, double> getLineParams(const cv::Vec4i &line);

        bool isOnLine(const cv::Vec4i &line, double x, double y);

    };
}

#endif //DEMO_ANGLEDETECOR_H