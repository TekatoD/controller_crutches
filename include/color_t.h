#pragma once


namespace drwn {
    class color_t {
    public:
        color_t() = default;

        color_t(int r, int g, int b);

        int get_red() const noexcept;

        void set_red(int r) noexcept;

        int get_green() const noexcept;

        void set_green(int g) noexcept;

        int get_blue() const noexcept;

        void set_blue(int b) noexcept;

        bool operator==(const color_t& rhs) const;

        bool operator!=(const color_t& rhs) const;

    private:
        int m_r{0};
        int m_g{0};
        int m_b{0};
    };
}


