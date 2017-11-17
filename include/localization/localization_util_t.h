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
        static bool intersect_lines(const line_t &l1, const line_t &l2, point2d_t &intersection, float eps = 0.00001);
        
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
        static constexpr float DEFAULT_FIELD_WIDTH = 6000.0f;
        static constexpr float DEFAULT_FIELD_HEIGHT = 4000.0f;
        static constexpr float DEFAULT_GATE_HEIGHT = 1400.0f;
        static constexpr float DEFAULT_PENALTY_WIDTH = 600.0f;
        static constexpr float DEFAULT_PENALTY_HEIGHT = 2200.0f;
        
        field_map_t();
        ~field_map_t();

        void initialize_field();
        void log_field_lines() const;

        std::tuple<line_type_t, point2d_t> intersect_with_field(const line_t &l, float minDist = 0.0f);

        void set_field_width(float field_width) {
            if (m_debug) LOG_DEBUG << "Field width: " << field_width;
            m_config.field_width = field_width;
        }
        float get_field_width() const { return m_config.field_height; }

        void set_field_height(float field_height) {
            if (m_debug) LOG_DEBUG << "Field height: " << field_height;
            m_config.field_height = field_height;
        }
        float get_field_height() const { return m_config.field_height; }

        void set_gate_height(float gate_height) {
            if (m_debug) LOG_DEBUG << "Gate height: " << gate_height;
            m_config.gate_height = gate_height;
        }
        float get_gate_height() const { return m_config.gate_height;}

        void set_penalty_width(float penalty_width) {
            if (m_debug) LOG_DEBUG << "Penalty width: " << penalty_width;
            m_config.penalty_width = penalty_width;
        }
        float get_penalty_width() const { return m_config.penalty_width; }

        void set_penalty_height(float penalty_height) {
            if (m_debug) LOG_DEBUG << "Penalty height: " << penalty_height;
            m_config.penalty_height = penalty_height;
        }
        float get_penalty_height() const { return m_config.penalty_height; }


        bool is_debug_enabled() const { return m_debug; }
        void enable_debug(bool debug) { m_debug = debug; }
    private:
        struct config_t {
            float field_width, field_height;
            float gate_height;
            float penalty_width, penalty_height;

            config_t()
            : field_width(DEFAULT_FIELD_WIDTH), field_height(DEFAULT_FIELD_HEIGHT),
              gate_height(DEFAULT_GATE_HEIGHT),
              penalty_width(DEFAULT_PENALTY_WIDTH), penalty_height(DEFAULT_PENALTY_HEIGHT)
            {

            }
        } m_config;

        bool m_debug {true};

        std::map<line_type_t, line_t> m_fieldLines;

        void make_lines(float fw, float fh, float pw, float ph, float gh);
    };

};

#endif