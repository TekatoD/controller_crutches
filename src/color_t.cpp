#include "color_t.h"

drwn::color_t::color_t(int r, int g, int b)
        : m_r(r), m_g(g), m_b(b) {}

int drwn::color_t::get_red() const noexcept {
    return m_r;
}

void drwn::color_t::set_red(int r) noexcept {
    color_t::m_r = r;
}

int drwn::color_t::get_green() const noexcept {
    return m_g;
}

void drwn::color_t::set_green(int g) noexcept {
    color_t::m_g = g;
}

int drwn::color_t::get_blue() const noexcept {
    return m_b;
}

void drwn::color_t::set_blue(int b) noexcept {
    color_t::m_b = b;
}
