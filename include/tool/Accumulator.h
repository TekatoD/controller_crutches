#pragma once


namespace drwn {
    class Accumulator {
    public:
        Accumulator() = default;

        Accumulator(const Accumulator&) = default;

        Accumulator& operator=(const Accumulator&) = default;

        explicit Accumulator(unsigned int threshold);

        Accumulator& operator=(Accumulator&&) = delete;

        Accumulator& operator++();

        Accumulator operator++(int);

        operator bool() const;

        void Increment();

        void Reset();

        unsigned int GetThreshold() const;

        void SetThreshold(unsigned int threshold);

        bool IsOverflow() const noexcept;

        unsigned int GetStep() const;

        void SetStep(unsigned int step);

    private:
        unsigned int m_threshold{0};
        unsigned int m_counter{0};
        unsigned int m_step{1};
    };
}


