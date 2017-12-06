/**
 *  @autor tekatod
 *  @date 4/28/17
 */
#include "motion/pose2d_t.h"
#include <math.h>
#include <boost/math/constants/constants.hpp>
#include <math/angle_tools.h>

drwn::pose2d_t::pose2d_t() : m_x(0), m_y(0), m_theta(0) { }

drwn::pose2d_t::pose2d_t(float x, float y, float theta) : m_x(x), m_y(y), m_theta(theta) {
    this->normalize_theta();
}

float drwn::pose2d_t::get_x() const {
    return m_x;
}

float drwn::pose2d_t::get_y() const {
    return m_y;
}

float drwn::pose2d_t::get_theta() const {
    return m_theta;
}

void drwn::pose2d_t::set_x(float x) {
    m_x = x;
}

void drwn::pose2d_t::set_y(float y) {
    m_y = y;
}

void drwn::pose2d_t::set_theta(float theta) {
    m_theta = theta;
    this->normalize_theta();
}

bool drwn::pose2d_t::is_nan() const {
    return std::isnan(m_x) || std::isnan(m_y) || std::isnan(m_theta);
}

void drwn::pose2d_t::normalize_theta() {
    using namespace boost::math;
//    while (m_theta < 0) m_theta += 2 * constants::pi<float>();
//    m_theta = fmod(m_theta, 2 * constants::pi<float>()) - constants::pi<float>();

//    while (m_theta > constants::pi<float>()) {
//        m_theta -= 2 * constants::pi<float>();
//    }
//    while (m_theta < -constants::pi<float>()) {
//        m_theta += 2 * constants::pi<float>();
//    }

    constexpr float pi = constants::pi<float>();
    m_theta = m_theta-2*pi*std::floor( (m_theta+pi)/(2*pi) );
}

drwn::pose2d_t drwn::pose2d_t::operator+(const drwn::pose2d_t &rhs) const {
    pose2d_t result(*this);
    result += rhs;
    return result;
}

drwn::pose2d_t& drwn::pose2d_t::operator+=(const drwn::pose2d_t &rhs) {
    m_x += rhs.m_x;
    m_y += rhs.m_y;
    m_theta += rhs.m_theta;
    this->normalize_theta();
    return *this;
}

drwn::pose2d_t drwn::pose2d_t::operator-(const drwn::pose2d_t &rhs) const {
    pose2d_t result(*this);
    result -= rhs;
    return result;
}

drwn::pose2d_t& drwn::pose2d_t::operator-=(const drwn::pose2d_t &rhs) {
    m_x -= rhs.m_x;
    m_y -= rhs.m_y;
    m_theta -= rhs.m_theta;
    this->normalize_theta();
    return *this;
}


void drwn::pose2d_t::rotate_around(const drwn::pose2d_t& pose) {
    float c = cos(pose.get_theta());
    float s = sin(pose.get_theta());
    float d_x = m_x - pose.get_x();
    float d_y = m_y - pose.get_y();
    m_x = c * (d_x) - s * (d_y) + pose.get_x();
    m_y = s * (d_x) + c * (d_y) + pose.get_y();
}

namespace drwn {
    std::ostream& operator<<(std::ostream &os, const drwn::pose2d_t &data) {
        os << " " << data.m_x << " " << data.m_y << " " << degrees(data.m_theta);
        return os;
    }
}

