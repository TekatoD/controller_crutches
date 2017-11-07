/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>
#include <gamecontroller/game_controller_t.h>

#include "state_machine_t.h"
#include "motion/modules/head_t.h"
#include "motion/modules/action_t.h"
#include "motion/modules/walking_t.h"
#include "motion/motion_status_t.h"
#include "motion/motion_manager_t.h"

using namespace drwn;

state_machine_t::state_machine_t() {
    m_old_btn = 0;
    m_is_started = false;
    m_role = ROLE_IDLE;
}


void state_machine_t::check(CM730_t* cm730) {
    if (motion_status_t::FALLEN != STANDUP && m_is_started == 1) {
        walking_t::get_instance()->stop();
        while (walking_t::get_instance()->is_running()) usleep(8000);

        action_t::get_instance()->joint.set_enable_body(true, true);

        if (motion_status_t::FALLEN == FORWARD)
            action_t::get_instance()->start(10);   // FORWARD GETUP
        else if (motion_status_t::FALLEN == BACKWARD)
            action_t::get_instance()->start(11);   // BACKWARD GETUP

        while (action_t::get_instance()->is_running()) usleep(8000);

        head_t::get_instance()->joint.set_enable_head_only(true, true);
        walking_t::get_instance()->joint.set_enable_body_without_head(true, true);
    }

    if (m_old_btn == motion_status_t::BUTTON)
        return;

    m_old_btn = motion_status_t::BUTTON;

    if (m_old_btn & BTN_START) {
        if (m_is_started) {
            disable();
        } else {
            enable();
        }
    }

    if (m_old_btn & BTN_STATE) {
        if (!m_is_started) {
            bool penalize_switched = false;
            switch (m_role) {
                case ROLE_IDLE:
                    m_role = ROLE_SOCCER;
                    penalize_switched = false;
                    break;
                case ROLE_SOCCER:
                    m_role = ROLE_PENALTY;
                    penalize_switched = true;
                    break;
                case ROLE_PENALTY:
                    m_role = ROLE_GOALKEEPER;
                    penalize_switched = true;
                    break;
                case ROLE_GOALKEEPER:
                default:
                    m_role = ROLE_IDLE;
                    break;
            }

            usleep(10000);

            if (penalize_switched) {
                if (m_role == ROLE_PENALTY) {
                    game_controller_t::get_instance()->send_penalise();
                } else {
                    game_controller_t::get_instance()->send_unpenalise();
                }
            }

            update_leds(cm730);
        } else {
            disable();
        }
    }
}

void state_machine_t::update_leds(CM730_t* cm730) const {
    switch (m_role) {
        case (ROLE_SOCCER):
            cm730->write_byte(CM730_t::P_LED_PANNEL, 0x01, NULL);
            break;
        case (ROLE_PENALTY):
            cm730->write_byte(CM730_t::P_LED_PANNEL, 0x02, NULL);
            break;
        case (ROLE_GOALKEEPER):
            cm730->write_byte(CM730_t::P_LED_PANNEL, 0x04, NULL);
            break;
        case (ROLE_IDLE):
        default:
            cm730->write_byte(CM730_t::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);
            break;
    }
}

void state_machine_t::enable() {
    m_is_started = true;
    motion_manager_t::get_instance()->reinitialize();
    motion_manager_t::get_instance()->set_enable(true);

    action_t::get_instance()->joint.set_enable_body(true, true);

    action_t::get_instance()->start(9);
    while (action_t::get_instance()->is_running()) usleep(8000);

    head_t::get_instance()->joint.set_enable_head_only(true, true);
    walking_t::get_instance()->joint.set_enable_body_without_head(true, true);

    motion_manager_t::get_instance()->reset_gyro_calibration();
    while (true) {
        if (motion_manager_t::get_instance()->get_calibration_status() == 1) {
            break;
        } else if (motion_manager_t::get_instance()->get_calibration_status() == -1) {
            motion_manager_t::get_instance()->reset_gyro_calibration();
        }
        usleep(8000);
    }
}

void state_machine_t::disable() {
    m_is_started = false;
    walking_t::get_instance()->stop();
    action_t::get_instance()->joint.set_enable_body(true, true);

    while (!action_t::get_instance()->start(15)) usleep(8000);
    while (action_t::get_instance()->is_running()) usleep(8000);
}

int state_machine_t::is_started() {
    return m_is_started;
}

const pose_2D_t& state_machine_t::get_spawn_position() const {
    return m_spawn_pos;
}

const pose_2D_t& state_machine_t::get_starting_position() const {
    return m_starting_pos;
}

role state_machine_t::get_role() const {
    return m_role;
}

void state_machine_t::set_role(role role) {
    m_role = role;
}

void state_machine_t::set_spawn_position(const pose_2D_t& pos) {
    m_spawn_pos = pos;
}

void state_machine_t::set_starting_position(const pose_2D_t& pos) {
    m_starting_pos = pos;
}
