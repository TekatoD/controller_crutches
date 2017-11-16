/*
 *   Matrix.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include "vector3d_t.h"
#include "point_t.h"


namespace drwn {
    class matrix3d_t {
    public:
        enum {
            m00 = 0,
            m01,
            m02,
            m03,
            m10,
            m11,
            m12,
            m13,
            m20,
            m21,
            m22,
            m23,
            m30,
            m31,
            m32,
            m33,
            MAXNUM_ELEMENT
        };

    private:

    protected:

    public:
        float m[MAXNUM_ELEMENT]; // Element

        matrix3d_t();

        matrix3d_t(const matrix3d_t& mat);

        ~matrix3d_t();

        void identity();

        bool inverse();

        void scale(vector3d_t scale);

        void rotate(float angle, vector3d_t axis);

        void translate(vector3d_t offset);

        point3d_t transform(point3d_t point);

        vector3d_t transform(vector3d_t vector);

        void set_transform(point3d_t point, vector3d_t angle);

        matrix3d_t& operator=(const matrix3d_t& mat);

        matrix3d_t& operator*=(const matrix3d_t& mat);

        matrix3d_t operator*(const matrix3d_t& mat);
    };
}