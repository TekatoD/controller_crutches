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
    this->process_cv();
}

void soccer_behavior_t::process_cv() {
    if (m_debug) LOG_INFO << "SOCCER BEHAVIOR: Processing CV...";
    m_camera->update_image();
    cv::Mat frame = m_camera->get_image();
    m_vision->set_frame(frame);
    m_vision->process();
}

soccer_behavior_t::~soccer_behavior_t() {

}
