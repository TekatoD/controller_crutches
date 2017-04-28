/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once


#include <ostream>

namespace Robot {
    class OdoData {
    public:

        OdoData();

        OdoData(double x, double y, double theta);

        double getX() const;

        double getY() const;

        double getTheta() const;

        void setX(double x);

        void setY(double y);

        void setTheta(double theta);

        void normalizeTheta();

        friend std::ostream& operator << (std::ostream& os, const OdoData& data);

    private:
        double m_x;
        double m_y;
        double m_theta;
    };
}

