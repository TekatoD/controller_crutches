#include "tool/accumulator_t.h"

drwn::accumulator_t::accumulator_t(unsigned int threshold)
        : m_threshold(threshold) {}

unsigned int drwn::accumulator_t::get_threshold() const {
    return m_threshold;
}

void drwn::accumulator_t::set_threshold(unsigned int threshold) {
    m_threshold = threshold;
}

void drwn::accumulator_t::reset() {
    m_counter = 0;
}

void drwn::accumulator_t::increment() {
    if (!is_overflow()) {
        m_counter += m_step;
    }
}

drwn::accumulator_t drwn::accumulator_t::operator++(int) {
    accumulator_t tmp(*this);
    increment();
    return tmp;
}

drwn::accumulator_t& drwn::accumulator_t::operator++() {
    increment();
    return *this;
}

bool drwn::accumulator_t::is_overflow() const noexcept {
    return m_counter >= m_threshold;
}

drwn::accumulator_t::operator bool() const {
    return is_overflow();
}

unsigned int drwn::accumulator_t::get_step() const {
    return m_step;
}

void drwn::accumulator_t::set_step(unsigned int step) {
    m_step = step;
}
