/*
 *   Vector.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include "point_t.h"

namespace drwn {
    class vector_3D_t {
    private:

    protected:

    public:
        float X;
        float Y;
        float Z;

        vector_3D_t();

        vector_3D_t(float x, float y, float z);

        vector_3D_t(const point_3D_t& pt1, const point_3D_t& pt2);

        vector_3D_t(const vector_3D_t& vector);

        ~vector_3D_t();

        float length();

        void normalize();

        float dot(const vector_3D_t& vector);

        vector_3D_t cross(const vector_3D_t& vector);

        float angle_between(vector_3D_t& vector);

        float angle_between(vector_3D_t& vector, vector_3D_t& axis);

        vector_3D_t& operator=(const vector_3D_t& vector);

        vector_3D_t& operator+=(const vector_3D_t& vector);

        vector_3D_t& operator-=(const vector_3D_t& vector);

        vector_3D_t& operator+=(const float value);

        vector_3D_t& operator-=(const float value);

        vector_3D_t& operator*=(const float value);

        vector_3D_t& operator/=(const float value);

        vector_3D_t operator+(const vector_3D_t& vector);

        vector_3D_t operator-(const vector_3D_t& vector);

        vector_3D_t operator+(const float value);

        vector_3D_t operator-(const float value);

        vector_3D_t operator*(const float value);

        vector_3D_t operator/(const float value);
    };
}
