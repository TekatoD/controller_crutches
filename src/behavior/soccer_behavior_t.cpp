/// \autor arssivka
/// \date 11/13/17

#include "behavior/soccer_behavior_t.h"
#include <cv.hpp>
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
#include "behavior/ball_tracker_t.h"
#include "behavior/ball_searcher_t.h"
#include "behavior/ball_follower_t.h"
#include "behavior/go_to_t.h"

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
          m_game_controller(game_controller_t::get_instance()),
          m_field{field_map_t::get_instance()},
          m_tracker{ball_tracker_t::get_instance()},
          m_searcher{ball_searcher_t::get_instance()},
          m_follower{ball_follower_t::get_instance()},
          m_goto{go_to_t::get_instance()} {
    // Do nothing
}

void soccer_behavior_t::process() {
    this->process_buttons();
    if (m_behavior_active) {
        this->process_game_controller();
        this->process_cv();
        this->process_localization();
        this->process_decision();
    }
    this->check_rate();
}

void soccer_behavior_t::process_buttons() {
    m_buttons->update();
    bool update_rate = false;

    if (m_rate_buttons_check.is_passed()) {
        // Fist button activates and deactivates behavior
        if (m_buttons->is_button_pressed(buttons_t::FIRST_BUTTON)) {
            m_behavior_active = !m_behavior_active;
            m_walking->stop();
            m_kicking->stop();
            m_action->joint.set_enable_body(true, true);
            m_action->start(m_behavior_active ? 9 : 15);
            update_rate = true;
            if (m_debug) {
                if (m_behavior_active) {
                    LOG_DEBUG << "SOCCER BEHAVIOR: Activating behavior...";
                } else {
                    LOG_DEBUG << "SOCCER BEHAVIOR: Deactivating behavior...";
                }
            }
        }

        // Second button switches penalise state;
        if (m_buttons->is_button_pressed(buttons_t::SECOND_BUTTON)) {
            if (!m_manual_penalised) {
                m_game_controller->send_penalise();
                m_manual_penalised = true;
            } else {
                m_game_controller->send_unpenalise();
                m_manual_penalised = false;
            }
            update_rate = true;
        }
    }


    if (update_rate) {
        m_rate_buttons_check.update();
    }
}

void soccer_behavior_t::process_game_controller() {
    m_game_controller->update();
}

void soccer_behavior_t::process_decision() {
    auto normalize = [](float& theta) {
        while (theta < -180.0f) theta += 2.0f * 180.0f;
        while (theta > 180.0f) theta -= 2.0f * 180.0f;
    };

    if (!m_prepared) {
        if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Preparing...";
        m_action->joint.set_enable_body(true, true);
        m_action->start(m_behavior_active ? 9 : 15);
        m_prepared = true;
    }

    if (motion_status_t::fall_type != fall_type_t::STANDUP && !m_action->is_running()) {
        if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Getting up";
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
        // Wait while robot hasn't got up
        if (m_action->is_running() || !m_behavior_active) {
            if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Decision making skipped";
            return;
        }

        const auto& gc_data = m_game_controller->get_game_ctrl_data();
        const auto& odo = m_walking->get_odo();

        auto ball = m_vision->detect_ball();

        if (gc_data.state == STATE_SET || gc_data.state == STATE_READY || gc_data.state == STATE_INITIAL) {
            if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Ready state processing...";
            const pose2d_t starnig; // TODO Get starting position
            if (m_tracker->is_no_ball()) {
                m_head->move_to_home();
            }
            m_walking->set_odo(starnig);
            m_walking->stop();
            return;
        }

        if (gc_data.state == STATE_PLAYING) {
            if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Playing state processing...";
//        if (State.kickOffTeam != team) {
//            // TODO KickOff
//        }

//            if (m_PreviousState != STATE_SET) { // TODO Reset odometry
//                m_PreviousState = STATE_SET;
//                Walking::GetInstance()->SetOdo(Starting);
//            }
            point2d_t ball_point(-1, -1); // No ball
            if (ball != cv::Rect()) {
                // Adapt new ball to old tracker
                ball_point = point2d_t(ball.x + ball.width / 2.0f,
                                       ball.y + ball.height / 2.0f);
                m_LEDs->set_eye_led({0, 255, 0});
            } else {
                m_LEDs->set_eye_led({255, 0, 0});
            }

            // Switch to head and walking after action
            m_head->joint.set_enable_head_only(true, true);
            m_walking->joint.set_enable_body_without_head(true, true);
            m_tracker->process(ball_point);

            // Calculate angles to gate
            float free_space = (m_field->get_field_height() - m_field->get_gate_height()) / 2.0f;
            float y_top = m_field->get_field_height() - free_space;
            float y_bot = y_top - m_field->get_gate_height();

            float pan = motion_status_t::current_joints.get_angle(joint_data_t::ID_HEAD_PAN);
            float angle_top = degrees(atan2f(m_field->get_field_height() - odo.get_y(), y_top - odo.get_x()) -
                                      odo.get_theta());
            float angle_bot = degrees(atan2f(m_field->get_field_height() - odo.get_y(), y_bot - odo.get_x()) -
                                      odo.get_theta());
            angle_bot -= pan;
            angle_top -= pan;

            normalize(angle_bot);
            normalize(angle_top);

//            if (m_tracker->is_no_ball()) {
//                m_searcher->process();
//                return;
//            } else {
//                m_searcher->set_last_position(m_tracker->get_ball_position());
//            }
//
//            // Follow the ball
//            m_follower->process(m_tracker->get_ball_position(), angle_top, angle_bot);
//
//            // Kicking the ball
//            if (m_follower->get_kicking_action() != kicking_action_t::NO_KICKING) {
//                m_head->joint.set_enable_head_only(true, true);
//                m_action->joint.set_enable_body_without_head(true, true);
//                // Kick the ball
//                if (m_follower->get_kicking_action() == kicking_action_t::RIGHT_LEG_KICK) {
//                    m_action->start(12);   // RIGHT KICK
//                } else {
//                    m_action->start(13);   // LEFT KICK
//                }
//            }
        }
    }
}

void soccer_behavior_t::process_cv() {
    if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Processing CV...";
    try {
        m_camera->update_image();
        cv::Mat frame = m_camera->get_image();
        m_vision->set_frame(frame);
        m_vision->process();
    } catch (const image_source_failure& e) {
        LOG_WARNING << "SOCCER BEHAVIOR: CV loop iteration failed: " << e.what();
    }
}

void soccer_behavior_t::check_rate() {
    if (!m_rate_processing_behavior.is_passed()) {
        if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Too fast processing. Sleep...";
//        std::this_thread::sleep_until(m_rate_processing_behavior.get_next_time_point());
        m_rate_processing_behavior.update();
    }
}

void soccer_behavior_t::process_localization() {

}
