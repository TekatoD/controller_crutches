/**
 *  @autor arssivka
 *  @date 5/8/17
 */

#pragma once

#include <ostream>
#include "minIni.h"

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

    void LoadINISettings(minIni* ini);

    void LoadINISettings(minIni* ini, const std::string& section);

    void SaveINISettings(minIni* ini);

    void SaveINISettings(minIni* ini, const std::string& section);

    friend std::ostream& operator<<(std::ostream& os, const Field& field);

private:
    float m_FieldWidth;
    float m_FieldLength;
    float m_GateWidth;
};


