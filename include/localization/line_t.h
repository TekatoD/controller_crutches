#ifndef _LOCALIZATION_UTIL_H_
#define _LOCALIZATION_UTIL_H_

#include <cmath>
#include <map>
#include <ostream>
#include <log/trivial_logger_t.h>

#include <math/point_t.h>

/*
 *  Units: mm
 */

using namespace drwn;

namespace drwn {
    struct line_t {
        point2d_t p1, p2;

        line_t(float x1, float y1, float x2, float y2);

        line_t(const point2d_t& p1, const point2d_t& p2);

        float length() const;

        bool is_point_on_line(float px, float py, float eps = 0.001) const;

        static bool intersect_lines(const line_t& l1, const line_t& l2, point2d_t& intersection, float eps = 0.00001);

        friend std::ostream& operator<<(std::ostream& out, const line_t& l);
    };
};

#endif