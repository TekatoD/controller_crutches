/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#pragma once


#include <ostream>

namespace Robot {
    class Pose2D {
    public:

        Pose2D();

        Pose2D(double x, double y, double theta = 0);

        double getX() const;

        double getY() const;

        double getTheta() const;

        void setX(double x);

        void setY(double y);

        void setTheta(double theta);

        void normalizeTheta();

        Pose2D operator+(const Pose2D& rhs) const;

        Pose2D& operator+=(const Pose2D& rhs);

        Pose2D operator-(const Pose2D& rhs) const;

        Pose2D& operator-=(const Pose2D& rhs);

        friend std::ostream& operator << (std::ostream& os, const Pose2D& data);

    private:
        double m_x;
        double m_y;
        double m_theta;
    };
}

