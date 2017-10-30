#include "tool/Accumulator.h"

Robot::Accumulator::Accumulator(unsigned int threshold)
        : m_threshold(threshold) {}

unsigned int Robot::Accumulator::GetThreshold() const {
    return m_threshold;
}

void Robot::Accumulator::SetThreshold(unsigned int threshold) {
    m_threshold = threshold;
}

void Robot::Accumulator::Reset() {
    m_counter = 0;
}

void Robot::Accumulator::Increment() {
    if (!IsOverflow()) {
        m_counter += m_step;
    }
}

Robot::Accumulator Robot::Accumulator::operator++(int) {
    Accumulator tmp(*this);
    Increment();
    return tmp;
}

Robot::Accumulator& Robot::Accumulator::operator++() {
    Increment();
    return *this;
}

bool Robot::Accumulator::IsOverflow() const noexcept {
    return m_counter >= m_threshold;
}

Robot::Accumulator::operator bool() const {
    return IsOverflow();
}

unsigned int Robot::Accumulator::GetStep() const {
    return m_step;
}

void Robot::Accumulator::SetStep(unsigned int step) {
    m_step = step;
}
