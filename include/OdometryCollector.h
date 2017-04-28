/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once

#include "OdoData.h"

namespace Robot {
    class OdometryCollector {
    public:

        OdometryCollector();

        OdometryCollector(OdoData initial);

        OdometryCollector(double x, double y, double theta);

        void odoTranslate(OdoData offset);

        OdoData getPose() const;

    private:
        OdoData m_pose;
    };
}

