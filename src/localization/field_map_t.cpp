/// \autor arssivka
/// \date 11/17/17

#include <log/trivial_logger_t.h>
#include <math/point_t.h>
#include <localization/line_t.h>
#include "localization/field_map_t.h"

using namespace drwn;

field_map_t* field_map_t::get_instance() {
    static field_map_t instance;
    return &instance;
}

void field_map_t::set_field_width(float field_width) {
    if (m_debug) LOG_DEBUG << "FIELD: width = " << field_width;
    m_config.field_width = field_width;
}

void field_map_t::set_field_height(float field_height) {
    if (m_debug) LOG_DEBUG << "FIELD: height = " << field_height;
    m_config.field_height = field_height;
}

void field_map_t::set_gate_height(float gate_height) {
    if (m_debug) LOG_DEBUG << "FIELD: gate_height = " << gate_height;
    m_config.gate_height = gate_height;
}

void field_map_t::set_penalty_width(float penalty_width) {
    if (m_debug) LOG_DEBUG << "FIELD: penalty_width = " << penalty_width;
    m_config.penalty_width = penalty_width;
}

void field_map_t::set_penalty_height(float penalty_height) {
    if (m_debug) LOG_DEBUG << "FIELD: penalty_height = " << penalty_height;
    m_config.penalty_height = penalty_height;
}

bool field_map_t::is_debug_enabled() const { return m_debug; }

void field_map_t::enable_debug(bool debug) { m_debug = debug; }

field_map_t::field_map_t() {
    initialize_field();
}

void field_map_t::initialize_field() {
    make_lines(m_config.field_width, m_config.field_height, m_config.penalty_width, m_config.penalty_height,
               m_config.gate_height);
    if (m_debug) log_field_lines();
}


void field_map_t::log_field_lines() const {
    if (!m_debug) return;

    LOG_DEBUG << "Field lines";
    for (auto& kv : m_field_lines) {
        LOG_DEBUG << (int) kv.first << " : " << kv.second;
    }
}

std::tuple<field_map_t::line_type_t, point2d_t> field_map_t::intersect_with_field(
        const line_t& l, float min_dist) {
    line_type_t type = line_type_t::NONE;
    point2d_t isec(0.0f, 0.0f);
    float dist = MAX_DIST;

    for (auto& kv : m_field_lines) {
        line_type_t temp_type = kv.first;
        point2d_t temp_isec;
        if (line_t::intersect_lines(l, kv.second, temp_isec)) {
            if (l.is_point_on_line(temp_isec.X, temp_isec.Y)) {
                // line_t can intersect multiple lines on the field
                // Find the line with the closest intersection point
                // If minDist is specified, only accept intersections with this minimum distance
                float temp_dist = point2d_t::distance(l.p1, temp_isec);
                if (temp_dist < dist && temp_dist >= min_dist) {
                    type = temp_type;
                    isec = temp_isec;
                    dist = temp_dist;
                }
            }
        }
    }

    if (fabsf(dist - MAX_DIST) < 0.0001) {
        type = line_type_t::NONE;
    }

    return std::make_tuple(type, isec);
}

void field_map_t::make_lines(float fw, float fh, float pw, float ph, float gh) {
    // Middle of the field is the origin (0.0, 0.0)
    // x: left is negative, right is positive
    // y: bottom is negative, top is positive

    m_field_lines = {
            {line_type_t::CENTRAL_LINE,         line_t(0.0f, fh / 2.0f, 0.0f, -fh / 2.0f)},
            {line_type_t::FIELD_LEFT,           line_t(-fw / 2.0f, fh / 2.0f, -fw / 2.0f, -fh / 2.0f)},
            {line_type_t::FIELD_RIGHT,          line_t(fw / 2.0f, fh / 2.0f, fw / 2.0f, -fh / 2.0f)},
            {line_type_t::FIELD_TOP,            line_t(-fw / 2.0f, fh / 2.0f, fw / 2.0f, fh / 2.0f)},
            {line_type_t::FIELD_BOTTOM,         line_t(-fw / 2.0f, -fh / 2.0f, fw / 2.0f, -fh / 2.0f)},
            {line_type_t::PENALTY_LEFT_TOP,     line_t(-fw / 2.0f, ph / 2.0f, (-fw / 2.0f) + pw, ph / 2.0f)},
            {line_type_t::PENALTY_LEFT_BOTTOM,  line_t(-fw / 2.0f, -ph / 2.0f, (-fw / 2.0f) + pw, -ph / 2.0f)},
            {line_type_t::PENALTY_LEFT_HEIGHT,  line_t((-fw / 2.0f) + pw, ph / 2.0f, (-fw / 2.0f) + pw, -ph / 2.0f)},
            {line_type_t::PENALTY_RIGHT_TOP,    line_t((fw / 2.0f) - pw, ph / 2.0f, fw / 2.0f, ph / 2.0f)},
            {line_type_t::PENALTY_RIGHT_BOTTOM, line_t((fw / 2.0f) - pw, -ph / 2.0f, fw / 2.0f, -ph / 2.0f)},
            {line_type_t::PENALTY_RIGHT_HEIGHT, line_t((fw / 2.0f) - pw, ph / 2.0f, (fw / 2.0f) - pw, -ph / 2.0f)}
    };
}

float field_map_t::get_field_width() const { return m_config.field_height; }

float field_map_t::get_field_height() const { return m_config.field_height; }

float field_map_t::get_gate_height() const { return m_config.gate_height; }

float field_map_t::get_penalty_width() const { return m_config.penalty_width; }

float field_map_t::get_penalty_height() const { return m_config.penalty_height; }
