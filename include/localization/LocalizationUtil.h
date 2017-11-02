#ifndef _LOCALIZATION_UTIL_H_
#define _LOCALIZATION_UTIL_H_

#include <cmath>
#include <map>
#include <ostream>

#include <minIni.h>
#include <math/Point.h>

/*
 *  Units: meters
 */

using namespace Robot;

namespace Localization {
    float CrossPoints(const Point2D& p1, const Point2D& p2)
    {
        return p1.X * p2.Y - p1.Y * p2.X;
    }
    
    struct Line {
        Point2D p1, p2;
        
        Line(float x1, float y1, float x2, float y2)
            : p1(x1, y1), p2(x2, y2)
        {
        }
        
        Line(const Point2D& p1, const Point2D& p2)
            : p1(p1), p2(p2)
        {
        }
        
        float Length() const
        {
            return Point2D::Distance(p1, p2);
        }
        
        bool IsPointOnLine(float px, float py, float eps=0.001) const
        {
            Point2D point(px, py);
            float LengthTo = Line(p1, point).Length();
            float LengthFrom = Line(point, p2).Length();
            float LineLength = this->Length();
            
            bool Lengths = LengthTo + LengthFrom - LineLength < eps;
            bool xAxis = (px >= p1.X && px <= p2.X) || (px <= p1.X && px >= p2.X);
            bool yAxis = (py >= p1.Y && py <= p2.Y) || (py <= p1.Y && py >= p2.Y);
            
            return Lengths && xAxis && yAxis;
        }
        
        /*
         * http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
         */
        static bool IntersectLines(const Line& l1, const Line& l2, Point2D& intersection, float eps=0.00001)
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
            if (l1.IsPointOnLine(intersection.X, intersection.Y) && l2.IsPointOnLine(intersection.X, intersection.Y)) {
                return true;
            }
            
            // Point doesn't lie on both lines. No intersection
            return false;
        }
        
        friend std::ostream& operator<<(std::ostream& out, const Line& l);
    };
    
    std::ostream& operator<<(std::ostream& out, const Line& l)
    {
        out << "(" << l.p1.X << "," << l.p1.Y << ")->(" << l.p2.X << "," << l.p2.Y << ")";
        return out;
    }
    
    class FieldMap {
    public:
        enum class LineType {
            CENTRAL_LINE = 0,
            FIELD_LEFT = 1,
            FIELD_RIGHT = 2,
            FIELD_TOP = 3,
            FIELD_BOTTOM = 4,
            PENALTY_LEFT_TOP = 5,
            PENALTY_LEFT_BOTTOM = 6,
            PENALTY_LEFT_HEIGHT = 7,
            PENALTY_RIGHT_TOP = 8,
            PENALTY_RIGHT_BOTTOM = 9,
            PENALTY_RIGHT_HEIGHT = 10,
            NONE = 11
        };
        
        FieldMap()
        {
        }
        
        ~FieldMap() {}
        
        void LoadIniSettings(minIni* ini)
        {
            float fw, fh, pw, ph, gh;
            
            fw = ini->getf("NewField", "field_width");
            fh = ini->getf("NewField", "field_height");
            pw = ini->getf("NewField", "penalty_width");
            ph = ini->getf("NewField", "penalty_height");
            gh = ini->getf("NewField", "gate_height");
            
            makeLines(fw, fh, pw, ph, gh);
        }
        
        void PrintFieldLines() const
        {
            for (auto& kv : m_fieldLines) {
                std::cout << (int)kv.first << " : " << kv.second << std::endl;
            }
        }
        
        
        std::tuple<LineType, Point2D> IntersectWithField(const Line& l)
        {
            LineType type = LineType::NONE;
            Point2D isec(0.0f, 0.0f);
            float dist = l.Length() * 100;
            
            for (auto& kv : m_fieldLines) {
                LineType temp_type = kv.first;
                Point2D temp_isec;
                if (Line::IntersectLines(l, kv.second, temp_isec)) {
                    std::cout << (int)temp_type << "; " << temp_isec.X << ", " << temp_isec.Y << std::endl;
                    if (l.IsPointOnLine(temp_isec.X, temp_isec.Y)) {
                        // Line can intersect multiple lines on the field
                        // Find the line with the closest intersection point
                        float temp_dist = Point2D::Distance(l.p1, temp_isec);
                        if (temp_dist < dist) {
                            type = temp_type; 
                            isec = temp_isec;
                            dist = temp_dist;
                        }
                    }
                }
            }
            
            if (fabs(dist - l.Length() * 100) < 0.0001) {
                type = LineType::NONE;
            }
            
            return std::make_tuple(type, isec);
        }
        
    private:
        std::map<LineType, Line> m_fieldLines;
        
        // hmmmm
        void makeLines(float fw, float fh, float pw, float ph, float gh)
        {
            // Middle of the field is the origin (0.0, 0.0)
            // x: left is negative, right is positive
            // y: bottom is negative, top is positive
            m_fieldLines = {
                {LineType::CENTRAL_LINE, Line(0.0f, fh/2.0f, 0.0f, -fh/2.0f)},
                {LineType::FIELD_LEFT, Line(-fw/2.0f, fh/2.0f, -fw/2.0f, -fh/2.0f)},
                {LineType::FIELD_RIGHT, Line(fw/2.0f, fh/2.0f, fw/2.0f, -fh/2.0f)},
                {LineType::FIELD_TOP, Line(-fw/2.0f, fh/2.0f, fw/2.0f, fh/2.0f)},
                {LineType::FIELD_BOTTOM, Line(-fw/2.0f, -fh/2.0f, fw/2.0f, -fh/2.0f)},
                {LineType::PENALTY_LEFT_TOP, Line(-fw/2.0f, ph/2.0f, (-fw/2.0f)+pw, ph/2.0f)},
                {LineType::PENALTY_LEFT_BOTTOM, Line(-fw/2.0f, -ph/2.0f, (-fw/2.0f)+pw, -ph/2.0f)},
                {LineType::PENALTY_LEFT_HEIGHT, Line((-fw/2.0f)+pw, ph/2.0f, (-fw/2.0f)+pw, -ph/2.0f)},
                {LineType::PENALTY_RIGHT_TOP, Line((fw/2.0f)-pw, ph/2.0f, fw/2.0f, ph/2.0f)},
                {LineType::PENALTY_RIGHT_BOTTOM, Line((fw/2.0f)-pw, -ph/2.0f, fw/2.0f, -ph/2.0f)},
                {LineType::PENALTY_RIGHT_HEIGHT, Line((fw/2.0f)-pw, ph/2.0f, (fw/2.0f)-pw, -ph/2.0f)}
            };
        }
    };

};

#endif