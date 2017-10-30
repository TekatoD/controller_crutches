/**
 *  @autor arssivka
 *  @date 5/8/17
 */

#pragma once

#include <ostream>

#define FIELD_SECTION ("Field")
#define INVALID_VALUE   -1024.0

class Field {
public:
    float GetWidth() const;

    void SetWidth(float field_width);

    float GetLength() const;

    void SetLength(float field_height);

    float GetGateWidth() const;

    void SetGateWidth(float gate_width);

    friend std::ostream& operator<<(std::ostream& os, const Field& field);

private:
    float m_FieldWidth;
    float m_FieldLength;
    float m_GateWidth;
};


