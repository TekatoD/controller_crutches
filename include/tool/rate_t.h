#pragma once


#include <chrono>

namespace drwn {
    template<class Clock>
    class rate_t {
    public:
        using clock = Clock;
        using duration = typename Clock::duration;
        using time_point = typename Clock::time_point;

        explicit rate_t(duration dur = duration(),
                      time_point tp = time_point())
                : m_dur(dur), m_next_tp(tp) {}

        duration get_duration() const {
            return m_dur;
        }

        void set_duration(duration duration) {
            m_dur = duration;
        }

        time_point get_next_time_point() const {
            return m_next_tp;
        }

        void set_next_time_point(time_point tp) {
            m_next_tp = tp;
        }

        void update() {
            m_next_tp = clock::now() + m_dur;
        }

        bool is_passed() const noexcept {
            return clock::now() >= m_next_tp;
        }

    private:
        duration m_dur;
        time_point m_next_tp;
    };

    using system_rate_t = rate_t<std::chrono::system_clock>;
    using steady_rate_t = rate_t<std::chrono::steady_clock>;
    using high_resolution_rate_t = rate_t<std::chrono::high_resolution_clock>;
}


