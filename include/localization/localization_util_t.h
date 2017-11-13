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

namespace localization {
    struct line_t {
        Point2D p1, p2;

        line_t(float x1, float y1, float x2, float y2);
        line_t(const Point2D& p1, const Point2D& p2);
        
        float length() const;
        bool is_point_on_line(float px, float py, float eps = 0.001) const;
        static bool intersect_lines(const line_t &l1, const line_t &l2, Point2D &intersection, float eps = 0.00001);
        
        friend std::ostream& operator<<(std::ostream& out, const line_t& l);
    };
    
    class field_map_t {
    public:
        enum class line_type_t {
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
        
        field_map_t();
        ~field_map_t();
        
        void load_ini_settings(minIni *ini);
        void print_field_lines() const;
        
        std::tuple<line_type_t, Point2D> intersect_with_field(const line_t &l, float minDist = 0.0f);
        
    private:
        std::map<line_type_t, line_t> m_fieldLines;
        
        void make_lines(float fw, float fh, float pw, float ph, float gh);
    };

};

#endif