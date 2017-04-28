/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "OdoData.h"
#include <math.h>

Robot::OdoData::OdoData() : m_x(0), m_y(0), m_theta(0) { }

Robot::OdoData::OdoData(double x, double y, double theta) : m_x(x), m_y(y), m_theta(theta) { }

double Robot::OdoData::getX() const {
    return m_x;
}

double Robot::OdoData::getY() const {
    return m_y;
}

double Robot::OdoData::getTheta() const {
    return m_theta;
}

void Robot::OdoData::setX(double x) {
    m_x = x;
}

void Robot::OdoData::setY(double y) {
    m_y = y;
}

void Robot::OdoData::setTheta(double theta) {
    m_theta = theta;
}

void Robot::OdoData::normalizeTheta() {
    double twoPi = 2 * M_PI;
    m_theta = m_theta - twoPi * floor( m_theta / twoPi );
}

namespace Robot {
    std::ostream &operator<<(std::ostream &os, const Robot::OdoData &data) {
        os << " " << data.m_x << " " << data.m_y << " " << data.m_theta;
        return os;
    }
}
