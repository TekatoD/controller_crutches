#pragma once


namespace drwn {
    class Color {
    public:
        Color() = default;

        Color(int r, int g, int b);

        int GetRed() const noexcept;

        void SetRed(int r) noexcept;

        int GetGreen() const noexcept;

        void SetGreen(int g) noexcept;

        int GetBlue() const noexcept;

        void SetBlue(int b) noexcept;

    private:
        int m_r{0};
        int m_g{0};
        int m_b{0};
    };
}


