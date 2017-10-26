/*
 * StatusCheck.h
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#pragma once

#include "hw/CM730.h"
#include "Pose2D.h"

#define CNTRL_SECTION "State Machine"

namespace Robot {
    enum {
        BTN_STATE = 1,
        BTN_START = 2
    };

    enum Role {
        ROLE_IDLE = 0,
        ROLE_SOCCER = 1,
        ROLE_PENALTY = 2,
        ROLE_GOALKEEPER = 3,
        ROLES_COUNT
    };

    class StateMachine {
    private:
        StateMachine();

        int m_old_btn;
        Role m_role;
        bool m_manual_penalty;
        bool m_is_started;

        Pose2D m_spawn_pos;
        Pose2D m_starting_pos;

    public:
        static StateMachine* GetInstance() {
            static StateMachine instance;
            return &instance;
        }

        int IsStarted();

        const Pose2D& GetSpawnPosition() const;

        const Pose2D& GetStartingPosition() const;

        void SetSpawnPosition(const Pose2D& pos);

        void SetStartingPosition(const Pose2D& pos);

        Role GetRole() const;

        void SetRole(Role role);

        void Check(CM730* cm730);

        void Disable();

        void Enable();

        void UpdateLeds(CM730* cm730) const;
    };
}

