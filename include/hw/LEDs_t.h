#pragma once


#include <hw/CM730_t.h>
#include "color_t.h"

namespace drwn {
    class LEDs_t {
    public:
        static LEDs_t* get_instance();

        void initialize(CM730_t* cm730) noexcept;

        void set_panel_led(bool first, bool second, bool third);

        void set_head_led(const color_t& color);

        void set_eye_led(const color_t& color);

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        LEDs_t() = default;

        void check_CM730();

    private:
        bool m_debug{false};

        color_t m_head_color{-1, -1, -1};
        color_t m_eye_color{-1, -1, -1};
        int m_mask = -1;

        CM730_t* m_cm730{nullptr};
    };
}


