#pragma once


namespace Robot {
    template<class Clock>
    class Rate {
    public:
        using Duration = typename Clock::duration;
        using TimePoint = typename Clock::time_point;

        explicit Rate(Duration dur = Duration(),
                      TimePoint tp = TimePoint())
                : m_dur(dur), m_next_tp(tp) {}

        Duration GetDuration() const {
            return m_dur;
        }

        void SetDuration(Duration duration) {
            m_dur = duration;
        }

        TimePoint GetNextTimePoint() const {
            return m_next_tp;
        }

        void SetNextTimePoint(TimePoint tp) {
            m_next_tp = tp;
        }

        void Update() {
            m_next_tp = Clock::now() + m_dur;
        }

        bool IsPassed() const noexcept {
            return Clock::now() >= m_next_tp;
        }

    private:
        Duration m_dur;
        TimePoint m_next_tp;
    };
}


