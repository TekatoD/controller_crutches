#include "Color.h"

int Robot::Color::GetRed() const noexcept {
    return r;
}

void Robot::Color::SetRed(int r) noexcept {
    Color::r = r;
}

int Robot::Color::GetGreen() const noexcept {
    return g;
}

void Robot::Color::SetGreen(int g) noexcept {
    Color::g = g;
}

int Robot::Color::GetBlue() const noexcept {
    return b;
}

void Robot::Color::SetBlue(int b) noexcept {
    Color::b = b;
}
