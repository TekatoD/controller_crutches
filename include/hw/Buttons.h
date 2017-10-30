#pragma once

#include <hw/CM730.h>

namespace Robot {
    class Buttons {
    public:
        enum ButtonId {
            FIRST_BUTTON,
            SECOND_BUTTON
        };

        static Buttons* GetInstance();

        void Update();

        bool IsButtonPressed(ButtonId btn) const;

        bool IsDebugEnabled() const noexcept;

        void EnableDebug(bool debug) noexcept;

    private:
        Buttons() = default;

    private:
        bool m_debug{false};

        bool m_button_status[2];
    };
}


