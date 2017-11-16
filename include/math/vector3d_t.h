/*
 *   Vector.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include "point_t.h"

namespace drwn {
    class vector3d_t {
    private:

    protected:

    public:
        float X;
        float Y;
        float Z;

        vector3d_t();

        vector3d_t(float x, float y, float z);

        vector3d_t(const point3d_t& pt1, const point3d_t& pt2);

        vector3d_t(const vector3d_t& vector);

        ~vector3d_t();

        float length();

        void normalize();

        float dot(const vector3d_t& vector);

        vector3d_t cross(const vector3d_t& vector);

        float angle_between(vector3d_t& vector);

        float angle_between(vector3d_t& vector, vector3d_t& axis);

        vector3d_t& operator=(const vector3d_t& vector);

        vector3d_t& operator+=(const vector3d_t& vector);

        vector3d_t& operator-=(const vector3d_t& vector);

        vector3d_t& operator+=(const float value);

        vector3d_t& operator-=(const float value);

        vector3d_t& operator*=(const float value);

        vector3d_t& operator/=(const float value);

        vector3d_t operator+(const vector3d_t& vector);

        vector3d_t operator-(const vector3d_t& vector);

        vector3d_t operator+(const float value);

        vector3d_t operator-(const float value);

        vector3d_t operator*(const float value);

        vector3d_t operator/(const float value);
    };
}
