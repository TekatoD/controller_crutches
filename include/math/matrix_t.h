/*
 *   Matrix.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include "vector_3D_t.h"
#include "point_t.h"


namespace drwn {
    class matrix_3D_t {
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

        matrix_3D_t();

        matrix_3D_t(const matrix_3D_t& mat);

        ~matrix_3D_t();

        void identity();

        bool inverse();

        void scale(vector_3D_t scale);

        void rotate(float angle, vector_3D_t axis);

        void translate(vector_3D_t offset);

        point_3D_t transform(point_3D_t point);

        vector_3D_t transform(vector_3D_t vector);

        void set_transform(point_3D_t point, vector_3D_t angle);

        matrix_3D_t& operator=(const matrix_3D_t& mat);

        matrix_3D_t& operator*=(const matrix_3D_t& mat);

        matrix_3D_t operator*(const matrix_3D_t& mat);
    };
}