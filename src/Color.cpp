#include "Color.h"

drwn::Color::Color(int r, int g, int b)
        : m_r(r), m_g(g), m_b(b) {}

int drwn::Color::GetRed() const noexcept {
    return m_r;
}

void drwn::Color::SetRed(int r) noexcept {
    Color::m_r = r;
}

int drwn::Color::GetGreen() const noexcept {
    return m_g;
}

void drwn::Color::SetGreen(int g) noexcept {
    Color::m_g = g;
}

int drwn::Color::GetBlue() const noexcept {
    return m_b;
}

void drwn::Color::SetBlue(int b) noexcept {
    Color::m_b = b;
}
