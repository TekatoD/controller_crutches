/*
 *   Point.cpp
 *   represents a point in 2D
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "math/point_t.h"

using namespace drwn;


point_2D_t::point_2D_t() {
    X = 0;
    Y = 0;
}


point_2D_t::point_2D_t(float x, float y) {
    X = x;
    Y = y;
}


point_2D_t::point_2D_t(const point_2D_t& point) {
    X = point.X;
    Y = point.Y;
}


point_2D_t::~point_2D_t() {
}


/*returns the euclidian distance between pt1 and pt2*/
float point_2D_t::Distance(point_2D_t& pt1, point_2D_t& pt2) {
    float x = pt1.X - pt2.X;
    float y = pt1.Y - pt2.Y;
    return sqrt(x * x + y * y);
}

float point_2D_t::Distance(const point_2D_t& pt1, const point_2D_t& pt2) {
    float x = pt1.X - pt2.X;
    float y = pt1.Y - pt2.Y;
    return sqrt(x * x + y * y);
}

point_2D_t& point_2D_t::operator=(const point_2D_t& point) {
    X = point.X;
    Y = point.Y;
    return *this;
}


point_2D_t& point_2D_t::operator+=(const point_2D_t& point) {
    X += point.X;
    Y += point.Y;
    return *this;
}


point_2D_t& point_2D_t::operator-=(const point_2D_t& point) {
    X -= point.X;
    Y -= point.Y;
    return *this;
}


point_2D_t& point_2D_t::operator+=(float value) {
    X += value;
    Y += value;
    return *this;
}


point_2D_t& point_2D_t::operator-=(float value) {
    X -= value;
    Y -= value;
    return *this;
}


point_2D_t& point_2D_t::operator*=(float value) {
    X *= value;
    Y *= value;
    return *this;
}


point_2D_t& point_2D_t::operator/=(float value) {
    X /= value;
    Y /= value;
    return *this;
}


point_2D_t point_2D_t::operator+(const point_2D_t& point) {
    return point_2D_t(X + point.X, Y + point.Y);
}


point_2D_t point_2D_t::operator-(const point_2D_t& point) {
    return point_2D_t(X - point.X, Y - point.Y);
}


point_2D_t point_2D_t::operator+(float value) {
    return point_2D_t(X + value, Y + value);
}


point_2D_t point_2D_t::operator-(float value) {
    return point_2D_t(X - value, Y - value);
}


point_2D_t point_2D_t::operator*(float value) {
    return point_2D_t(X * value, Y * value);
}


point_2D_t point_2D_t::operator/(float value) {
    return point_2D_t(X / value, Y / value);
}


point_3D_t::point_3D_t() {
    X = 0;
    Y = 0;
    Z = 0;
}


point_3D_t::point_3D_t(float x, float y, float z) {
    X = x;
    Y = y;
    Z = z;
}


point_3D_t::point_3D_t(const point_3D_t& point) {
    X = point.X;
    Y = point.Y;
    Z = point.Z;
}


point_3D_t::~point_3D_t() {
}


float point_3D_t::Distance(const point_3D_t& pt1, const point_3D_t& pt2) {
    float x = pt1.X - pt2.X;
    float y = pt1.Y - pt2.Y;
    float z = pt1.Z - pt2.Z;
    return sqrt(x * x + y * y + z * z);
}


point_3D_t& point_3D_t::operator=(const point_3D_t& point) {
    X = point.X;
    Y = point.Y;
    Z = point.Z;
    return *this;
}


point_3D_t& point_3D_t::operator+=(const point_3D_t& point) {
    X += point.X;
    Y += point.Y;
    Z += point.Z;
    return *this;
}


point_3D_t& point_3D_t::operator-=(const point_3D_t& point) {
    X -= point.X;
    Y -= point.Y;
    Z -= point.Z;
    return *this;
}


point_3D_t& point_3D_t::operator+=(float value) {
    X += value;
    Y += value;
    Z += value;
    return *this;
}


point_3D_t& point_3D_t::operator-=(float value) {
    X -= value;
    Y -= value;
    Z -= value;
    return *this;
}


point_3D_t& point_3D_t::operator*=(float value) {
    X *= value;
    Y *= value;
    Z *= value;
    return *this;
}


point_3D_t& point_3D_t::operator/=(float value) {
    X /= value;
    Y /= value;
    Z /= value;
    return *this;
}


point_3D_t point_3D_t::operator+(const point_3D_t& point) {
    return point_3D_t(X + point.X, Y + point.Y, Z + point.Z);
}


point_3D_t point_3D_t::operator-(const point_3D_t& point) {
    return point_3D_t(X - point.X, Y - point.Y, Z - point.Z);
}


point_3D_t point_3D_t::operator+(float value) {
    return point_3D_t(X + value, Y + value, Z + value);
}


point_3D_t point_3D_t::operator-(float value) {
    return point_3D_t(X - value, Y - value, Z - value);
}


point_3D_t point_3D_t::operator*(float value) {
    return point_3D_t(X * value, Y * value, Z * value);
}


point_3D_t point_3D_t::operator/(float value) {
    return point_3D_t(X / value, Y / value, Z / value);
}
