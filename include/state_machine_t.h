/*
 * StatusCheck.h
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#pragma once

#include "hw/CM730_t.h"
#include "pose_2D_t.h"

#define CNTRL_SECTION "State Machine"

namespace drwn {
    enum {
        BTN_STATE = 1,
        BTN_START = 2
    };

    enum role {
        ROLE_IDLE = 0,
        ROLE_SOCCER = 1,
        ROLE_PENALTY = 2,
        ROLE_GOALKEEPER = 3,
        ROLES_COUNT
    };

    class state_machine_t {
    private:
        state_machine_t();

        int m_old_btn;
        role m_role;
        bool m_manual_penalty;
        bool m_is_started;

        pose_2D_t m_spawn_pos;
        pose_2D_t m_starting_pos;

    public:
        static state_machine_t* get_instance() {
            static state_machine_t instance;
            return &instance;
        }

        int is_started();

        const pose_2D_t& get_spawn_position() const;

        const pose_2D_t& get_starting_position() const;

        void set_spawn_position(const pose_2D_t& pos);

        void set_starting_position(const pose_2D_t& pos);

        role get_role() const;

        void set_role(role role);

        void check(CM730_t* cm730);

        void disable();

        void enable();

        void update_leds(CM730_t* cm730) const;
    };
}

