/*
 *   Point.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once


namespace drwn {
    class point_2D_t {
    private:

    protected:

    public:
        float X;
        float Y;

        point_2D_t();

        point_2D_t(float x, float y);

        point_2D_t(const point_2D_t& point);

        ~point_2D_t();

        /*compute the euclidean distance between pt1 and pt2*/
        static float Distance(point_2D_t& pt1, point_2D_t& pt2);

        point_2D_t& operator=(const point_2D_t& point);

        point_2D_t& operator+=(const point_2D_t& point);

        point_2D_t& operator-=(const point_2D_t& point);

        point_2D_t& operator+=(float value);

        point_2D_t& operator-=(float value);

        point_2D_t& operator*=(float value);

        point_2D_t& operator/=(float value);

        point_2D_t operator+(const point_2D_t& point);

        point_2D_t operator-(const point_2D_t& point);

        point_2D_t operator+(float value);

        point_2D_t operator-(float value);

        point_2D_t operator*(float value);

        point_2D_t operator/(float value);
    };

    class point_3D_t {
    private:

    protected:

    public:
        float X;
        float Y;
        float Z;

        point_3D_t();

        point_3D_t(float x, float y, float z);

        point_3D_t(const point_3D_t& point);

        ~point_3D_t();

        /*compute the euclidean distance between pt1 and pt2*/
        static float Distance(const point_3D_t& pt1, const point_3D_t& pt2);

        point_3D_t& operator=(const point_3D_t& point);

        point_3D_t& operator+=(const point_3D_t& point);

        point_3D_t& operator-=(const point_3D_t& point);

        point_3D_t& operator+=(float value);

        point_3D_t& operator-=(float value);

        point_3D_t& operator*=(float value);

        point_3D_t& operator/=(float value);

        point_3D_t operator+(const point_3D_t& point);

        point_3D_t operator-(const point_3D_t& point);

        point_3D_t operator+(float value);

        point_3D_t operator-(float value);

        point_3D_t operator*(float value);

        point_3D_t operator/(float value);
    };
}

