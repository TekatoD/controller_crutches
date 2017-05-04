/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "Pose2D.h"
#include <math.h>

Robot::Pose2D::Pose2D() : m_x(0), m_y(0), m_theta(0) { }

Robot::Pose2D::Pose2D(double x, double y, double theta) : m_x(x), m_y(y), m_theta(theta) { }

double Robot::Pose2D::X() const {
    return m_x;
}

double Robot::Pose2D::Y() const {
    return m_y;
}

double Robot::Pose2D::Theta() const {
    return m_theta;
}

void Robot::Pose2D::setX(double x) {
    m_x = x;
}

void Robot::Pose2D::setY(double y) {
    m_y = y;
}

void Robot::Pose2D::setTheta(double theta) {
    m_theta = theta;
}

void Robot::Pose2D::normalizeTheta() {
    double twoPi = 2 * M_PI;
    m_theta = m_theta - twoPi * floor( m_theta / twoPi );
}

Robot::Pose2D Robot::Pose2D::operator+(const Robot::Pose2D &rhs) const {
    Pose2D result(*this);
    result += rhs;
    return result;
}

Robot::Pose2D& Robot::Pose2D::operator+=(const Robot::Pose2D &rhs) {
    m_x += rhs.m_x;
    m_y += rhs.m_y;
    m_theta += rhs.m_theta;
    return *this;
}

Robot::Pose2D Robot::Pose2D::operator-(const Robot::Pose2D &rhs) const {
    Pose2D result(*this);
    result -= rhs;
    return result;
}

Robot::Pose2D& Robot::Pose2D::operator-=(const Robot::Pose2D &rhs) {
    m_x -= rhs.m_x;
    m_y -= rhs.m_y;
    m_theta -= rhs.m_theta;
    return *this;
}

void Robot::Pose2D::normalizeThetaInPi() {
    m_theta = remainder(m_theta, M_PI);
}

namespace Robot {
    std::ostream &operator<<(std::ostream &os, const Robot::Pose2D &data) {
        os << " " << data.m_x << " " << data.m_y << " " << data.m_theta;
        return os;
    }
}
