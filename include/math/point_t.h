/*
 *   Point.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once


namespace drwn {
    class point2d_t {
    private:

    protected:

    public:
        float X;
        float Y;

        point2d_t();

        point2d_t(float x, float y);

        point2d_t(const point2d_t& point);

        ~point2d_t();

        /*compute the euclidean distance between pt1 and pt2*/
        static float Distance(point2d_t& pt1, point2d_t& pt2);
        static float Distance(const point2d_t& pt1, const point2d_t& pt2);

        point2d_t& operator=(const point2d_t& point);

        point2d_t& operator+=(const point2d_t& point);

        point2d_t& operator-=(const point2d_t& point);

        point2d_t& operator+=(float value);

        point2d_t& operator-=(float value);

        point2d_t& operator*=(float value);

        point2d_t& operator/=(float value);

        point2d_t operator+(const point2d_t& point);

        point2d_t operator-(const point2d_t& point);

        point2d_t operator+(float value);

        point2d_t operator-(float value);

        point2d_t operator*(float value);

        point2d_t operator/(float value);
    };

    class point3d_t {
    private:

    protected:

    public:
        float X;
        float Y;
        float Z;

        point3d_t();

        point3d_t(float x, float y, float z);

        point3d_t(const point3d_t& point);

        ~point3d_t();

        /*compute the euclidean distance between pt1 and pt2*/
        static float Distance(const point3d_t& pt1, const point3d_t& pt2);

        point3d_t& operator=(const point3d_t& point);

        point3d_t& operator+=(const point3d_t& point);

        point3d_t& operator-=(const point3d_t& point);

        point3d_t& operator+=(float value);

        point3d_t& operator-=(float value);

        point3d_t& operator*=(float value);

        point3d_t& operator/=(float value);

        point3d_t operator+(const point3d_t& point);

        point3d_t operator-(const point3d_t& point);

        point3d_t operator+(float value);

        point3d_t operator-(float value);

        point3d_t operator*(float value);

        point3d_t operator/(float value);
    };
}

