#include "Color.h"

Robot::Color::Color(int r, int g, int b)
        : m_r(r), m_g(g), m_b(b) {}

int Robot::Color::GetRed() const noexcept {
    return m_r;
}

void Robot::Color::SetRed(int r) noexcept {
    Color::m_r = r;
}

int Robot::Color::GetGreen() const noexcept {
    return m_g;
}

void Robot::Color::SetGreen(int g) noexcept {
    Color::m_g = g;
}

int Robot::Color::GetBlue() const noexcept {
    return m_b;
}

void Robot::Color::SetBlue(int b) noexcept {
    Color::m_b = b;
}
