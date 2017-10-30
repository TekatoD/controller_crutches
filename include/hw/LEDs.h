#pragma once


#include <hw/CM730.h>
#include "Color.h"

namespace Robot {
    class LEDs {
    public:
        static LEDs* GetInstance();

        void Initialize(CM730* cm730) noexcept;

        void SetPanelLed(bool first, bool second, bool third);

        void SetHeadLed(const Color& color);

        void SetEyeLed(const Color& color);

        bool IsDebugEnabled() const noexcept;

        void EnableDebug(bool debug) noexcept;

    private:
        LEDs() = default;

        void CheckCM730();

    private:
        bool m_debug{false};

        CM730* m_cm730{nullptr};
    };
}


