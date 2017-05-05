/*
 * StatusCheck.h
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#pragma once

#include "CM730.h"
#include "minIni.h"
#include "Pose2D.h"

#define CNTRL_SECTION "controller"

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

        const Pose2D& SpawnPosition();

        const Pose2D& StartingPosition();

        Role GetRole() const;

        void SetRole(Role role);

        void Check(CM730& cm730);

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

        void Disable();

        void Enable();

        void UpdateLeds(CM730 &cm730) const;
    };
}

