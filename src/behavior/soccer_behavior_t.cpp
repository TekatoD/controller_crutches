/// \autor arssivka
/// \date 11/13/17

#include "behavior/soccer_behavior_t.h"
#include <cv.hpp>
#include <log/trivial_logger_t.h>
#include <hw/buttons_t.h>
#include <hw/LEDs_t.h>
#include <hw/camera_t.h>
#include <vision/vision_t.h>
#include <motion/modules/head_t.h>
#include <motion/modules/walking_t.h>
#include <motion/modules/action_t.h>
#include <motion/modules/kicking_t.h>
#include <game_controller/game_controller_t.h>
#include <motion/motion_status_t.h>
#include <hw/image_source_failure.h>
#include <thread>

using namespace drwn;

soccer_behavior_t::soccer_behavior_t()
        : m_camera(camera_t::get_instance()),
          m_vision(vision_t::get_instance()),
          m_head(head_t::get_instance()),
          m_walking(walking_t::get_instance()),
          m_action(action_t::get_instance()),
          m_kicking(kicking_t::get_instance()),
          m_localization(nullptr), // TODO Getter for localization!
          m_buttons(buttons_t::get_instance()),
          m_LEDs(LEDs_t::get_instance()),
          m_game_controller(game_controller_t::get_instance()) {
    // Do nothing
}

void soccer_behavior_t::process() {
    this->process_buttons();
    this->process_game_controller();
    this->process_cv();
    this->process_decision();
    this->check_rate();
}

void soccer_behavior_t::process_buttons() {
    m_buttons->update();

    if (m_rate_buttons_check.is_passed() &&
        m_buttons->is_button_pressed(buttons_t::FIRST_BUTTON)) {
        m_behavior_active = !m_behavior_active;
        m_rate_buttons_check.update();
    }
}

void soccer_behavior_t::process_game_controller() {
    m_game_controller->update();
}

void soccer_behavior_t::process_decision() {
    if (motion_status_t::fall_type != fall_type_t::STANDUP && !m_action->is_running()) {
        if (m_debug) LOG_INFO << "SOCCER BEHAVIOR: Getting up";
        m_state = state_t::STANDING_UP;
        m_walking->stop();
        m_kicking->stop();
        // Start standing up motion
        action_t::get_instance()->joint.set_enable_body(true, true);
        if (motion_status_t::fall_type == fall_type_t::FORWARD) {
            m_action->start(10); // Forward get up
        } else {
            m_action->start(11); // Backward get up
        }
    } else {
        // Wait while robot isn't up
        if (m_state == state_t::STANDING_UP && m_action->is_running()) {
            return;
        }


    }
}

void soccer_behavior_t::process_cv() {
    if (m_debug) LOG_INFO << "SOCCER BEHAVIOR: Processing CV...";
    try {
        m_camera->update_image();
        cv::Mat frame = m_camera->get_image();
        m_vision->set_frame(frame);
        m_vision->process();
    } catch(const image_source_failure& e) {
        LOG_WARNING << "SOCCER BEHAVIOR: CV loop iteration failed: " << e.what();
    }
}

soccer_behavior_t::~soccer_behavior_t() {
}

void soccer_behavior_t::check_rate() {
    if (!m_rate_processing_behavior.is_passed()) {
        if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Too fast processing. Sleep...";
        std::this_thread::sleep_until(m_rate_processing_behavior.get_next_time_point());
        m_rate_processing_behavior.update();
    }
}
