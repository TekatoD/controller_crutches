/**
 *  @autor arssivka
 *  @date 5/8/17
 */

#pragma once

#include <ostream>

#define FIELD_SECTION ("Field")
#define INVALID_VALUE   -1024.0

class field_t {
public:
    float get_width() const;

    void set_width(float field_width);

    float get_length() const;

    void set_length(float field_height);

    float get_gate_width() const;

    void set_gate_width(float gate_width);

    friend std::ostream& operator<<(std::ostream& os, const field_t& field);

private:
    float m_field_width;
    float m_field_length;
    float m_gate_width;
};


