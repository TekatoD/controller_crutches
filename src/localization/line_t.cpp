#include <iostream>
#include "localization/line_t.h"

drwn::line_t::line_t(float x1, float y1, float x2, float y2)
        : p1(x1, y1), p2(x2, y2) {
}

drwn::line_t::line_t(const point2d_t& p1, const point2d_t& p2)
        : p1(p1), p2(p2) {
}

float drwn::line_t::length() const {
    return point2d_t::distance(p1, p2);
}

bool drwn::line_t::is_point_on_line(float px, float py, float eps) const {
    point2d_t point(px, py);
    float LengthTo = line_t(p1, point).length();
    float LengthFrom = line_t(point, p2).length();
    float LineLength = this->length();

    bool Lengths = LengthTo + LengthFrom - LineLength < eps;
    bool xAxis = (px >= p1.X && px <= p2.X) || (px <= p1.X && px >= p2.X);
    bool yAxis = (py >= p1.Y && py <= p2.Y) || (py <= p1.Y && py >= p2.Y);

    return Lengths && xAxis && yAxis;
}

/*
 * http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
 */
bool drwn::line_t::intersect_lines(const line_t& l1, const line_t& l2, point2d_t& intersection, float eps) {
    float num1, num2, denom, ua, ub;
    float x1, x2, x3, x4, y1, y2, y3, y4;
    //l1 
    x1 = l1.p1.X;
    y1 = l1.p1.Y;
    x2 = l1.p2.X;
    y2 = l1.p2.Y;
    //l2
    x3 = l2.p1.X;
    y3 = l2.p1.Y;
    x4 = l2.p2.X;
    y4 = l2.p2.Y;

    num1 = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
    num2 = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

    // Parallel lines
    if (fabs(denom) < eps) {
        return false;
    }

    ua = num1 / denom;
    ub = num2 / denom;

    // Coincidental lines
    if ((fabs(ua) < eps) || (fabs(ub) < eps)) {
        return false;
    }

    // Substitute either ua or ub into the equation
    // Should work with ub,  but correct result is only with ua
    intersection.X = x1 + ua * (x2 - x1);
    intersection.Y = y1 + ua * (y2 - y1);

    // Check intersection point if it lies on both lines
    if (l1.is_point_on_line(intersection.X, intersection.Y) && l2.is_point_on_line(intersection.X, intersection.Y)) {
        return true;
    }

    // Point doesn't lie on both lines. No intersection
    return false;
}

namespace drwn {

    std::ostream& operator<<(std::ostream& out, const line_t& l) {
        out << "(" << l.p1.X << "," << l.p1.Y << ")->(" << l.p2.X << "," << l.p2.Y << ")";
        return out;
    }

};