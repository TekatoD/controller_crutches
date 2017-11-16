/*
 *   Vector.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "math/vector3d_t.h"

using namespace drwn;


vector3d_t::vector3d_t() {
    X = 0;
    Y = 0;
    Z = 0;
}


vector3d_t::vector3d_t(float x, float y, float z) {
    X = x;
    Y = y;
    Z = z;
}


vector3d_t::vector3d_t(const point3d_t& pt1, const point3d_t& pt2) {
    X = pt2.X - pt1.X;
    Y = pt2.Y - pt1.Y;
    Z = pt2.Z - pt1.Z;
}


vector3d_t::vector3d_t(const vector3d_t& vector) {
    X = vector.X;
    Y = vector.Y;
    Z = vector.Z;
}


vector3d_t::~vector3d_t() {
}


float vector3d_t::length() {
    return sqrt(X * X + Y * Y + Z * Z);
}


void vector3d_t::normalize() {
    float length = this->length();

    X = X / length;
    Y = Y / length;
    Z = Z / length;
}


float vector3d_t::dot(const vector3d_t& vector) {
    return (X * vector.X + Y * vector.Y + Z * vector.Z);
}


vector3d_t vector3d_t::cross(const vector3d_t& vector) {
    vector3d_t res;
    res.X = Y * vector.Z - Z * vector.Y;
    res.Y = Z * vector.X - X * vector.Z;
    res.Z = X * vector.Y - Y * vector.X;
    return res;
}


float vector3d_t::angle_between(vector3d_t& vector) {
    return acos((X * vector.X + Y * vector.Y + Z * vector.Z) / (length() * vector.length())) * (180.0 / 3.141592);
}


float vector3d_t::angle_between(vector3d_t& vector, vector3d_t& axis) {
    float angle = angle_between(vector);
    vector3d_t cross = this->cross(vector);
    if (cross.dot(axis) < 0.0)
        angle *= -1.0;

    return angle;
}


vector3d_t& vector3d_t::operator=(const vector3d_t& vector) {
    X = vector.X;
    Y = vector.Y;
    Z = vector.Z;
    return *this;
}


vector3d_t& vector3d_t::operator+=(const vector3d_t& vector) {
    X += vector.X;
    Y += vector.Y;
    Z += vector.Z;
    return *this;
}


vector3d_t& vector3d_t::operator-=(const vector3d_t& vector) {
    X -= vector.X;
    Y -= vector.Y;
    Z -= vector.Z;
    return *this;
}


vector3d_t& vector3d_t::operator+=(const float value) {
    X += value;
    Y += value;
    Z += value;
    return *this;
}


vector3d_t& vector3d_t::operator-=(const float value) {
    X -= value;
    Y -= value;
    Z -= value;
    return *this;
}


vector3d_t& vector3d_t::operator*=(const float value) {
    X *= value;
    Y *= value;
    Z *= value;
    return *this;
}


vector3d_t& vector3d_t::operator/=(const float value) {
    X /= value;
    Y /= value;
    Z /= value;
    return *this;
}


vector3d_t vector3d_t::operator+(const vector3d_t& vector) {
    return vector3d_t(X + vector.X, Y + vector.Y, Z + vector.Z);
}


vector3d_t vector3d_t::operator-(const vector3d_t& vector) {
    return vector3d_t(X - vector.X, Y - vector.Y, Z - vector.Z);
}


vector3d_t vector3d_t::operator+(const float value) {
    return vector3d_t(X + value, Y + value, Z + value);
}


vector3d_t vector3d_t::operator-(const float value) {
    return vector3d_t(X - value, Y - value, Z - value);
}


vector3d_t vector3d_t::operator*(const float value) {
    return vector3d_t(X * value, Y * value, Z * value);
}


vector3d_t vector3d_t::operator/(const float value) {
    return vector3d_t(X / value, Y / value, Z / value);
}
