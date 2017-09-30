/*
 *   Vector.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _VECTOR_H_
#define _VECTOR_H_

#include "Point.h"

namespace Robot {
    class Vector3D {
    private:

    protected:

    public:
        float X;
        float Y;
        float Z;

        Vector3D();

        Vector3D(float x, float y, float z);

        Vector3D(const Point3D& pt1, const Point3D& pt2);

        Vector3D(const Vector3D& vector);

        ~Vector3D();

        float Length();

        void Normalize();

        float Dot(const Vector3D& vector);

        Vector3D Cross(const Vector3D& vector);

        float AngleBetween(Vector3D& vector);

        float AngleBetween(Vector3D& vector, Vector3D& axis);

        Vector3D& operator=(const Vector3D& vector);

        Vector3D& operator+=(const Vector3D& vector);

        Vector3D& operator-=(const Vector3D& vector);

        Vector3D& operator+=(const float value);

        Vector3D& operator-=(const float value);

        Vector3D& operator*=(const float value);

        Vector3D& operator/=(const float value);

        Vector3D operator+(const Vector3D& vector);

        Vector3D operator-(const Vector3D& vector);

        Vector3D operator+(const float value);

        Vector3D operator-(const float value);

        Vector3D operator*(const float value);

        Vector3D operator/(const float value);
    };
}

#endif