//
// Created by nikitas on 20.03.16.
//

#pragma once

#include "opencv2/opencv.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace drwn {
    class BaseDetector {
    public:
        explicit BaseDetector(const std::string &detector_name = "");

        const std::string &DetectorName();

    private:
        const std::string m_detector_name;
    };

}
