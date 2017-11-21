/*
 *   Point.cpp
 *   represents a point in 2D
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "math/point_t.h"

using namespace drwn;


point2d_t::point2d_t() {
    X = 0;
    Y = 0;
}


point2d_t::point2d_t(float x, float y) {
    X = x;
    Y = y;
}


point2d_t::point2d_t(const point2d_t& point) {
    X = point.X;
    Y = point.Y;
}


point2d_t::~point2d_t() {
}


/*returns the euclidian distance between pt1 and pt2*/
float point2d_t::distance(point2d_t& pt1, point2d_t& pt2) {
    float x = pt1.X - pt2.X;
    float y = pt1.Y - pt2.Y;
    return sqrt(x * x + y * y);
}

float point2d_t::distance(const point2d_t& pt1, const point2d_t& pt2) {
    float x = pt1.X - pt2.X;
    float y = pt1.Y - pt2.Y;
    return sqrtf(x * x + y * y);
}

point2d_t& point2d_t::operator=(const point2d_t& point) {
    X = point.X;
    Y = point.Y;
    return *this;
}


point2d_t& point2d_t::operator+=(const point2d_t& point) {
    X += point.X;
    Y += point.Y;
    return *this;
}


point2d_t& point2d_t::operator-=(const point2d_t& point) {
    X -= point.X;
    Y -= point.Y;
    return *this;
}


point2d_t& point2d_t::operator+=(float value) {
    X += value;
    Y += value;
    return *this;
}


point2d_t& point2d_t::operator-=(float value) {
    X -= value;
    Y -= value;
    return *this;
}


point2d_t& point2d_t::operator*=(float value) {
    X *= value;
    Y *= value;
    return *this;
}


point2d_t& point2d_t::operator/=(float value) {
    X /= value;
    Y /= value;
    return *this;
}


point2d_t point2d_t::operator+(const point2d_t& point) {
    return point2d_t(X + point.X, Y + point.Y);
}


point2d_t point2d_t::operator-(const point2d_t& point) {
    return point2d_t(X - point.X, Y - point.Y);
}


point2d_t point2d_t::operator+(float value) {
    return point2d_t(X + value, Y + value);
}


point2d_t point2d_t::operator-(float value) {
    return point2d_t(X - value, Y - value);
}


point2d_t point2d_t::operator*(float value) {
    return point2d_t(X * value, Y * value);
}


point2d_t point2d_t::operator/(float value) {
    return point2d_t(X / value, Y / value);
}


point3d_t::point3d_t() {
    X = 0;
    Y = 0;
    Z = 0;
}


point3d_t::point3d_t(float x, float y, float z) {
    X = x;
    Y = y;
    Z = z;
}


point3d_t::point3d_t(const point3d_t& point) {
    X = point.X;
    Y = point.Y;
    Z = point.Z;
}


point3d_t::~point3d_t() {
}


float point3d_t::distance(const point3d_t& pt1, const point3d_t& pt2) {
    float x = pt1.X - pt2.X;
    float y = pt1.Y - pt2.Y;
    float z = pt1.Z - pt2.Z;
    return sqrt(x * x + y * y + z * z);
}


point3d_t& point3d_t::operator=(const point3d_t& point) {
    X = point.X;
    Y = point.Y;
    Z = point.Z;
    return *this;
}


point3d_t& point3d_t::operator+=(const point3d_t& point) {
    X += point.X;
    Y += point.Y;
    Z += point.Z;
    return *this;
}


point3d_t& point3d_t::operator-=(const point3d_t& point) {
    X -= point.X;
    Y -= point.Y;
    Z -= point.Z;
    return *this;
}


point3d_t& point3d_t::operator+=(float value) {
    X += value;
    Y += value;
    Z += value;
    return *this;
}


point3d_t& point3d_t::operator-=(float value) {
    X -= value;
    Y -= value;
    Z -= value;
    return *this;
}


point3d_t& point3d_t::operator*=(float value) {
    X *= value;
    Y *= value;
    Z *= value;
    return *this;
}


point3d_t& point3d_t::operator/=(float value) {
    X /= value;
    Y /= value;
    Z /= value;
    return *this;
}


point3d_t point3d_t::operator+(const point3d_t& point) {
    return point3d_t(X + point.X, Y + point.Y, Z + point.Z);
}


point3d_t point3d_t::operator-(const point3d_t& point) {
    return point3d_t(X - point.X, Y - point.Y, Z - point.Z);
}


point3d_t point3d_t::operator+(float value) {
    return point3d_t(X + value, Y + value, Z + value);
}


point3d_t point3d_t::operator-(float value) {
    return point3d_t(X - value, Y - value, Z - value);
}


point3d_t point3d_t::operator*(float value) {
    return point3d_t(X * value, Y * value, Z * value);
}


point3d_t point3d_t::operator/(float value) {
    return point3d_t(X / value, Y / value, Z / value);
}
