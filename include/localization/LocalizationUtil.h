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
    struct Line {
        Point2D p1, p2;
        
        Line(float x1, float y1, float x2, float y2);
        Line(const Point2D& p1, const Point2D& p2);
        
        float Length() const;
        bool IsPointOnLine(float px, float py, float eps=0.001) const;
        static bool IntersectLines(const Line& l1, const Line& l2, Point2D& intersection, float eps=0.00001);
        
        friend std::ostream& operator<<(std::ostream& out, const Line& l);
    };
    
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
        
        static constexpr float MAX_DIST = 12000.0f;
        
        FieldMap();
        ~FieldMap();
        
        void LoadIniSettings(minIni* ini);
        void PrintFieldLines() const;
        
        std::tuple<LineType, Point2D> IntersectWithField(const Line& l, float minDist=0.0f);
        
    private:
        std::map<LineType, Line> m_fieldLines;
        
        void makeLines(float fw, float fh, float pw, float ph, float gh);
    };

};

#endif