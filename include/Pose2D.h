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

        Pose2D(float x, float y, float theta = 0);

        float X() const;

        float Y() const;

        float Theta() const;

        void setX(float x);

        void setY(float y);

        void setTheta(float theta);

        void normalizeTheta();

        void rotateAround(const Pose2D& pose);

        Pose2D operator+(const Pose2D& rhs) const;

        Pose2D& operator+=(const Pose2D& rhs);

        Pose2D operator-(const Pose2D& rhs) const;

        Pose2D& operator-=(const Pose2D& rhs);

        friend std::ostream& operator << (std::ostream& os, const Pose2D& data);
        
    private:
        float m_x;
        float m_y;
        float m_theta;
    };
}

