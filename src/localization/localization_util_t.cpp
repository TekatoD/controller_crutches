#include <iostream>

#include "localization/localization_util_t.h"


localization::line_t::line_t(float x1, float y1, float x2, float y2)
: p1(x1, y1), p2(x2, y2)
{
}

localization::line_t::line_t(const Point2D& p1, const Point2D& p2)
: p1(p1), p2(p2)
{
}

float localization::line_t::length() const
{
    return Point2D::Distance(p1, p2);
}

bool localization::line_t::is_point_on_line(float px, float py, float eps) const
{
    Point2D point(px, py);
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
bool localization::line_t::intersect_lines(const line_t &l1, const line_t &l2, Point2D &intersection, float eps)
{
    float num1, num2, denom, ua, ub;
    float x1, x2, x3, x4, y1, y2, y3, y4;
    //l1 
    x1 = l1.p1.X; y1 = l1.p1.Y;
    x2 = l1.p2.X; y2 = l1.p2.Y;
    //l2
    x3 = l2.p1.X; y3 = l2.p1.Y;
    x4 = l2.p2.X; y4 = l2.p2.Y;
    
    num1 = (x4-x3)*(y1-y3)-(y4-y3)*(x1-x3);
    num2 = (x2-x1)*(y1-y3)-(y2-y1)*(x1-x3);
    denom = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1);
    
    // Parallel lines
    if (fabs(denom) < eps) {
        return false;
    }
    
    ua = num1 / denom;
    ub = num2 / denom;
    
    // Coincidental lines
    if ((fabs(ua) < eps) || (fabs(ub) < eps))  {
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

namespace localization {

    std::ostream& operator<<(std::ostream& out, const line_t& l)
    {
        out << "(" << l.p1.X << "," << l.p1.Y << ")->(" << l.p2.X << "," << l.p2.Y << ")";
        return out;
    }

};

localization::field_map_t::field_map_t()
{
}

localization::field_map_t::~field_map_t() {}

void localization::field_map_t::load_ini_settings(minIni *ini)
{
    float fw, fh, pw, ph, gh;
    
    fw = ini->getf("NewField", "field_width");
    fh = ini->getf("NewField", "field_height");
    pw = ini->getf("NewField", "penalty_width");
    ph = ini->getf("NewField", "penalty_height");
    gh = ini->getf("NewField", "gate_height");

    make_lines(fw, fh, pw, ph, gh);
}

void localization::field_map_t::print_field_lines() const
{
    for (auto& kv : m_fieldLines) {
        std::cout << (int)kv.first << " : " << kv.second << std::endl;
    }
}


std::tuple<localization::field_map_t::line_type_t, Point2D> localization::field_map_t::intersect_with_field(
        const line_t &l, float minDist)
{
    line_type_t type = line_type_t::NONE;
    Point2D isec(0.0f, 0.0f);
    float dist = MAX_DIST;
    
    for (auto& kv : m_fieldLines) {
        line_type_t temp_type = kv.first;
        Point2D temp_isec;
        if (line_t::intersect_lines(l, kv.second, temp_isec)) {
            if (l.is_point_on_line(temp_isec.X, temp_isec.Y)) {
                // line_t can intersect multiple lines on the field
                // Find the line with the closest intersection point
                // If minDist is specified, only accept intersections with this minimum distance
                float temp_dist = Point2D::Distance(l.p1, temp_isec);
                if (temp_dist < dist && temp_dist >= minDist) {
                    type = temp_type; 
                    isec = temp_isec;
                    dist = temp_dist;
                }
            }
        }
    }
    
    if (fabs(dist - MAX_DIST) < 0.0001) {
        type = line_type_t::NONE;
    }
    
    return std::make_tuple(type, isec);
}

void localization::field_map_t::make_lines(float fw, float fh, float pw, float ph, float gh)
{
    // Middle of the field is the origin (0.0, 0.0)
    // x: left is negative, right is positive
    // y: bottom is negative, top is positive
    m_fieldLines = {
        {line_type_t::CENTRAL_LINE, line_t(0.0f, fh/2.0f, 0.0f, -fh/2.0f)},
        {line_type_t::FIELD_LEFT, line_t(-fw/2.0f, fh/2.0f, -fw/2.0f, -fh/2.0f)},
        {line_type_t::FIELD_RIGHT, line_t(fw/2.0f, fh/2.0f, fw/2.0f, -fh/2.0f)},
        {line_type_t::FIELD_TOP, line_t(-fw/2.0f, fh/2.0f, fw/2.0f, fh/2.0f)},
        {line_type_t::FIELD_BOTTOM, line_t(-fw/2.0f, -fh/2.0f, fw/2.0f, -fh/2.0f)},
        {line_type_t::PENALTY_LEFT_TOP, line_t(-fw/2.0f, ph/2.0f, (-fw/2.0f)+pw, ph/2.0f)},
        {line_type_t::PENALTY_LEFT_BOTTOM, line_t(-fw/2.0f, -ph/2.0f, (-fw/2.0f)+pw, -ph/2.0f)},
        {line_type_t::PENALTY_LEFT_HEIGHT, line_t((-fw/2.0f)+pw, ph/2.0f, (-fw/2.0f)+pw, -ph/2.0f)},
        {line_type_t::PENALTY_RIGHT_TOP, line_t((fw/2.0f)-pw, ph/2.0f, fw/2.0f, ph/2.0f)},
        {line_type_t::PENALTY_RIGHT_BOTTOM, line_t((fw/2.0f)-pw, -ph/2.0f, fw/2.0f, -ph/2.0f)},
        {line_type_t::PENALTY_RIGHT_HEIGHT, line_t((fw/2.0f)-pw, ph/2.0f, (fw/2.0f)-pw, -ph/2.0f)}
    };
}