/**
 *  @autor arssivka
 *  @date 5/8/17
 */

#include "Field.h"

float Field::GetWidth() const {
    return m_FieldWidth;
}

void Field::SetWidth(float field_width) {
    if (field_width < 0) field_width = 0;
    if (m_GateWidth > field_width) m_GateWidth = field_width;
    m_FieldWidth = field_width;
}

float Field::GetLength() const {
    return m_FieldLength;
}

void Field::SetLength(float field_height) {
    if (field_height < 0) field_height = 0;
    m_FieldLength = field_height;
}

float Field::GetGateWidth() const {
    return m_GateWidth;
}

void Field::SetGateWidth(float gate_width) {
    if (gate_width < 0) gate_width = 0;
    if (gate_width > m_FieldWidth) m_FieldLength = gate_width;
    m_GateWidth = gate_width;
}

std::ostream& operator<<(std::ostream& os, const Field& field) {
    os << "field width: " << field.m_FieldWidth << " field length: " << field.m_FieldLength << " gate width: "
       << field.m_GateWidth;
    return os;
}
