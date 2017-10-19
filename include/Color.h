#pragma once


namespace Robot {
    class Color {
    public:
        int GetRed() const noexcept;

        void SetRed(int r) noexcept;

        int GetGreen() const noexcept;

        void SetGreen(int g) noexcept;

        int GetBlue() const noexcept;

        void SetBlue(int b) noexcept;

    private:
        int r;
        int g;
        int b;
    };
}


