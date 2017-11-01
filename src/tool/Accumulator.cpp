#include "tool/Accumulator.h"

drwn::Accumulator::Accumulator(unsigned int threshold)
        : m_threshold(threshold) {}

unsigned int drwn::Accumulator::GetThreshold() const {
    return m_threshold;
}

void drwn::Accumulator::SetThreshold(unsigned int threshold) {
    m_threshold = threshold;
}

void drwn::Accumulator::Reset() {
    m_counter = 0;
}

void drwn::Accumulator::Increment() {
    if (!IsOverflow()) {
        m_counter += m_step;
    }
}

drwn::Accumulator drwn::Accumulator::operator++(int) {
    Accumulator tmp(*this);
    Increment();
    return tmp;
}

drwn::Accumulator& drwn::Accumulator::operator++() {
    Increment();
    return *this;
}

bool drwn::Accumulator::IsOverflow() const noexcept {
    return m_counter >= m_threshold;
}

drwn::Accumulator::operator bool() const {
    return IsOverflow();
}

unsigned int drwn::Accumulator::GetStep() const {
    return m_step;
}

void drwn::Accumulator::SetStep(unsigned int step) {
    m_step = step;
}
