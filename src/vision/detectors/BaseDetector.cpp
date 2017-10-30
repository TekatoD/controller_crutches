//
// Created by nikitas on 20.03.16.
//

#include "vision/detectors/BaseDetector.h"

namespace Robot {

    BaseDetector::BaseDetector(const std::string &detector_name) :
            m_detector_name(detector_name) { }

    const std::string &BaseDetector::DetectorName() {
        return m_detector_name;
    }

}

