#pragma once


namespace drwn {
    class accumulator_t {
    public:
        accumulator_t() = default;

        accumulator_t(const accumulator_t&) = default;

        accumulator_t& operator=(const accumulator_t&) = default;

        explicit accumulator_t(unsigned int threshold);

        accumulator_t& operator=(accumulator_t&&) = delete;

        accumulator_t& operator++();

        accumulator_t operator++(int);

        operator bool() const;

        void increment();

        void reset();

        unsigned int get_threshold() const;

        void set_threshold(unsigned int threshold);

        bool is_overflow() const noexcept;

        unsigned int get_step() const;

        void set_step(unsigned int step);

    private:
        unsigned int m_threshold{0};
        unsigned int m_counter{0};
        unsigned int m_step{1};
    };
}


