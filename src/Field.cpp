/**
 *  @autor arssivka
 *  @date 5/8/17
 */

#include "Field.h"

double Field::GetWidth() const {
    return m_FieldWidth;
}

void Field::SetWidth(double field_width) {
    if (field_width < 0) field_width = 0;
    if (m_GateWidth > field_width) m_GateWidth = field_width;
    m_FieldWidth = field_width;
}

double Field::GetLength() const {
    return m_FieldLength;
}

void Field::SetLength(double field_height) {
    if (field_height < 0) field_height = 0;
    m_FieldLength = field_height;
}

double Field::GetGateWidth() const {
    return m_GateWidth;
}

void Field::SetGateWidth(double gate_width) {
    if (gate_width < 0) gate_width = 0;
    if (gate_width > m_FieldWidth) m_FieldLength = gate_width;
    m_GateWidth = gate_width;
}

void Field::LoadINISettings(minIni *ini) {
    LoadINISettings(ini, FIELD_SECTION);
}


void Field::LoadINISettings(minIni *ini, const std::string &section) {
    double value = -2;

    if ((value = ini->getd(section, "field_width", INVALID_VALUE)) != INVALID_VALUE) SetWidth(value);
    if ((value = ini->getd(section, "field_length", INVALID_VALUE)) != INVALID_VALUE) SetLength(value);
    if ((value = ini->getd(section, "gate_width", INVALID_VALUE)) != INVALID_VALUE) SetGateWidth(value);
}


void Field::SaveINISettings(minIni *ini) {
    SaveINISettings(ini, FIELD_SECTION);
}


void Field::SaveINISettings(minIni *ini, const std::string &section) {
    ini->put(section, "field_width", m_FieldWidth);
    ini->put(section, "field_length", m_FieldLength);
    ini->put(section, "gate_width", m_GateWidth);
}

std::ostream& operator<<(std::ostream& os, const Field& field) {
    os << "field width: " << field.m_FieldWidth << " field length: " << field.m_FieldLength << " gate width: "
       << field.m_GateWidth;
    return os;
}
