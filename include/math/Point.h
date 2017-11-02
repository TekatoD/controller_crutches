/*
 *   Point.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _POINT_H_
#define _POINT_H_


namespace Robot {
    class Point2D {
    private:

    protected:

    public:
        float X;
        float Y;

        Point2D();

        Point2D(float x, float y);

        Point2D(const Point2D& point);

        ~Point2D();

        /*compute the euclidean distance between pt1 and pt2*/
        static float Distance(Point2D& pt1, Point2D& pt2);
        static float Distance(const Point2D& pt1, const Point2D& pt2);

        Point2D& operator=(const Point2D& point);

        Point2D& operator+=(const Point2D& point);

        Point2D& operator-=(const Point2D& point);

        Point2D& operator+=(float value);

        Point2D& operator-=(float value);

        Point2D& operator*=(float value);

        Point2D& operator/=(float value);

        Point2D operator+(const Point2D& point);

        Point2D operator-(const Point2D& point);

        Point2D operator+(float value);

        Point2D operator-(float value);

        Point2D operator*(float value);

        Point2D operator/(float value);
    };

    class Point3D {
    private:

    protected:

    public:
        float X;
        float Y;
        float Z;

        Point3D();

        Point3D(float x, float y, float z);

        Point3D(const Point3D& point);

        ~Point3D();

        /*compute the euclidean distance between pt1 and pt2*/
        static float Distance(const Point3D& pt1, const Point3D& pt2);

        Point3D& operator=(const Point3D& point);

        Point3D& operator+=(const Point3D& point);

        Point3D& operator-=(const Point3D& point);

        Point3D& operator+=(float value);

        Point3D& operator-=(float value);

        Point3D& operator*=(float value);

        Point3D& operator/=(float value);

        Point3D operator+(const Point3D& point);

        Point3D operator-(const Point3D& point);

        Point3D operator+(float value);

        Point3D operator-(float value);

        Point3D operator*(float value);

        Point3D operator/(float value);
    };
}

#endif
