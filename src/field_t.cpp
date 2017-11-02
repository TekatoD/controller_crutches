/**
 *  @autor arssivka
 *  @date 5/8/17
 */

#include "field_t.h"

float field_t::get_width() const {
    return m_field_width;
}

void field_t::set_width(float field_width) {
    if (field_width < 0) field_width = 0;
    if (m_gate_width > field_width) m_gate_width = field_width;
    m_field_width = field_width;
}

float field_t::get_length() const {
    return m_field_length;
}

void field_t::set_length(float field_height) {
    if (field_height < 0) field_height = 0;
    m_field_length = field_height;
}

float field_t::get_gate_width() const {
    return m_gate_width;
}

void field_t::set_gate_width(float gate_width) {
    if (gate_width < 0) gate_width = 0;
    if (gate_width > m_field_width) m_field_length = gate_width;
    m_gate_width = gate_width;
}

std::ostream& operator<<(std::ostream& os, const field_t& field) {
    os << "field width: " << field.m_field_width << " field length: " << field.m_field_length << " gate width: "
       << field.m_gate_width;
    return os;
}
