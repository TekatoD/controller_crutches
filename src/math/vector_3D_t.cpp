/*
 *   Vector.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "math/vector_3D_t.h"

using namespace drwn;


vector_3D_t::vector_3D_t() {
    X = 0;
    Y = 0;
    Z = 0;
}


vector_3D_t::vector_3D_t(float x, float y, float z) {
    X = x;
    Y = y;
    Z = z;
}


vector_3D_t::vector_3D_t(const point_3D_t& pt1, const point_3D_t& pt2) {
    X = pt2.X - pt1.X;
    Y = pt2.Y - pt1.Y;
    Z = pt2.Z - pt1.Z;
}


vector_3D_t::vector_3D_t(const vector_3D_t& vector) {
    X = vector.X;
    Y = vector.Y;
    Z = vector.Z;
}


vector_3D_t::~vector_3D_t() {
}


float vector_3D_t::length() {
    return sqrt(X * X + Y * Y + Z * Z);
}


void vector_3D_t::normalize() {
    float length = this->length();

    X = X / length;
    Y = Y / length;
    Z = Z / length;
}


float vector_3D_t::dot(const vector_3D_t& vector) {
    return (X * vector.X + Y * vector.Y + Z * vector.Z);
}


vector_3D_t vector_3D_t::cross(const vector_3D_t& vector) {
    vector_3D_t res;
    res.X = Y * vector.Z - Z * vector.Y;
    res.Y = Z * vector.X - X * vector.Z;
    res.Z = X * vector.Y - Y * vector.X;
    return res;
}


float vector_3D_t::angle_between(vector_3D_t& vector) {
    return acos((X * vector.X + Y * vector.Y + Z * vector.Z) / (length() * vector.length())) * (180.0 / 3.141592);
}


float vector_3D_t::angle_between(vector_3D_t& vector, vector_3D_t& axis) {
    float angle = angle_between(vector);
    vector_3D_t cross = this->cross(vector);
    if (cross.dot(axis) < 0.0)
        angle *= -1.0;

    return angle;
}


vector_3D_t& vector_3D_t::operator=(const vector_3D_t& vector) {
    X = vector.X;
    Y = vector.Y;
    Z = vector.Z;
    return *this;
}


vector_3D_t& vector_3D_t::operator+=(const vector_3D_t& vector) {
    X += vector.X;
    Y += vector.Y;
    Z += vector.Z;
    return *this;
}


vector_3D_t& vector_3D_t::operator-=(const vector_3D_t& vector) {
    X -= vector.X;
    Y -= vector.Y;
    Z -= vector.Z;
    return *this;
}


vector_3D_t& vector_3D_t::operator+=(const float value) {
    X += value;
    Y += value;
    Z += value;
    return *this;
}


vector_3D_t& vector_3D_t::operator-=(const float value) {
    X -= value;
    Y -= value;
    Z -= value;
    return *this;
}


vector_3D_t& vector_3D_t::operator*=(const float value) {
    X *= value;
    Y *= value;
    Z *= value;
    return *this;
}


vector_3D_t& vector_3D_t::operator/=(const float value) {
    X /= value;
    Y /= value;
    Z /= value;
    return *this;
}


vector_3D_t vector_3D_t::operator+(const vector_3D_t& vector) {
    return vector_3D_t(X + vector.X, Y + vector.Y, Z + vector.Z);
}


vector_3D_t vector_3D_t::operator-(const vector_3D_t& vector) {
    return vector_3D_t(X - vector.X, Y - vector.Y, Z - vector.Z);
}


vector_3D_t vector_3D_t::operator+(const float value) {
    return vector_3D_t(X + value, Y + value, Z + value);
}


vector_3D_t vector_3D_t::operator-(const float value) {
    return vector_3D_t(X - value, Y - value, Z - value);
}


vector_3D_t vector_3D_t::operator*(const float value) {
    return vector_3D_t(X * value, Y * value, Z * value);
}


vector_3D_t vector_3D_t::operator/(const float value) {
    return vector_3D_t(X / value, Y / value, Z / value);
}
