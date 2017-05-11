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
    double GetWidth() const;

    void SetWidth(double field_width);

    double GetLength() const;

    void SetLength(double field_height);

    double GetGateWidth() const;

    void SetGateWidth(double gate_width);

    void LoadINISettings(minIni* ini);

    void LoadINISettings(minIni* ini, const std::string& section);

    void SaveINISettings(minIni* ini);

    void SaveINISettings(minIni* ini, const std::string& section);

    friend std::ostream& operator<<(std::ostream& os, const Field& field);

private:
    double m_FieldWidth;
    double m_FieldLength;
    double m_GateWidth;
};


