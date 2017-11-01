#pragma once

#include <hw/CM730_t.h>

namespace drwn {
    class buttons_t {
    public:
        enum button_id {
            FIRST_BUTTON,
            SECOND_BUTTON
        };

        static buttons_t* get_instance();

        void update();

        bool is_button_pressed(button_id btn) const;

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        buttons_t() = default;

    private:
        bool m_debug{false};

        bool m_button_status[2];
    };
}


