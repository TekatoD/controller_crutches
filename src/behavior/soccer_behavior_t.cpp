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
#include <localization/localization_t.h>
#include <thread>
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
          m_localization(localization_t::get_instance()),
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
    this->process_game_controller();
    if (m_behavior_active) {
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
    m_avoid_localization = false;

    if (!m_prepared) {
        if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Preparing...";
        m_action->joint.set_enable_body(true, true);
        m_action->start(m_behavior_active ? 9 : 15);
        m_prepared = true;
    }

    if (motion_status_t::fall_type != fall_type_t::STANDUP && !m_action->is_running()) {
        m_force_localization = true;

        auto last_pose = m_walking->get_odo();
        auto pose_dev = m_localization->get_calculated_pose_std_dev();

        m_localization->reset_pose_approximate_area_around(last_pose, pose_dev);

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
        if (m_action->is_running() || m_kicking->is_running() || !m_behavior_active) {
            if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Decision making skipped";
            m_avoid_localization = true;
            return;
        }

        const auto& gc_data = m_game_controller->get_game_ctrl_data();
        const auto& odo = m_walking->get_odo();
        
        // Update penalized flag
        int team_index = gc_data.teams[0].team_number == m_game_controller->get_team_number() ? 0 : 1;
        auto penalized = gc_data.teams[team_index].players[m_game_controller->get_team_number()].penalty;

        if (penalized && !m_penalized) {
            m_action->joint.set_enable_body(true, true);
            m_action->start(9);
            m_walking->set_x_move_amplitude(0.0f);
            m_walking->set_y_move_amplitude(0.0f);
            m_walking->set_a_move_amplitude(0.0f);
            m_walking->stop();
            m_penalized = true;

        }

        if (m_debug)
            LOG_DEBUG << "SOCCER BEHAVIOR: odo = ("
                      << odo.get_x() << ", "
                      << odo.get_y() << ", "
                      << degrees(odo.get_theta()) << ')';

        auto ball = m_vision->detect_ball();
//        m_ball_filter.set_ball(ball);
//        ball = m_ball_filter.get_ball();

        if (gc_data.state == STATE_INITIAL) {
            if (m_previous_state != STATE_INITIAL) {
                m_walking->set_odo(m_field->get_spawn_pose());
                m_localization->reset_current_pose(m_field->get_spawn_pose());
                m_previous_state = STATE_INITIAL;
            }
        }
        if (gc_data.state == STATE_READY) {
            if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Set state processing...";
            if (m_previous_state != STATE_READY) {
                m_walking->set_odo(m_field->get_spawn_pose());
                m_localization->reset_current_pose(m_field->get_spawn_pose());
                m_previous_state = STATE_READY;
            }
            m_goto->process(m_field->get_start_pose() - odo);
        } else if (gc_data.state == STATE_SET) {
            if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Ready state processing...";
            const pose2d_t starting = m_field->get_start_pose();
            if (m_tracker->is_no_ball()) {
                m_head->move_to_home();
            }
            if (m_previous_state != STATE_SET) {
                m_previous_state = STATE_SET;
                m_walking->set_odo(starting);
                m_localization->reset_current_pose(starting);
            }

            m_walking->stop();
            return;
        } else if (gc_data.state == STATE_PLAYING) {
            if (m_previous_state != STATE_PLAYING) {
                const pose2d_t starting = m_field->get_start_pose();
                m_walking->set_odo(starting);
                m_localization->reset_current_pose(starting);
                m_previous_state = STATE_PLAYING;
                m_ball_filter.reset();
            }
            
            if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Playing state processing...";
//        if (State.kickOffTeam != team) {
//            // TODO KickOff
//        }

//            if (m_PreviousState != STATE_SET) { // TODO Reset odometry
//                m_PreviousState = STATE_SET;
//                Walking::GetInstance()->SetOdo(Starting);
//            }
            color_t eye_leds{0, 255, 0};

            point2d_t ball_point(-1, -1); // No ball
            if (ball != cv::Rect()) {
                // Adapt new ball to old tracker
                ball_point = point2d_t(ball.x + ball.width / 2.0f,
                                       ball.y + ball.height);
            } else {
                eye_leds = color_t{255, 255, 0};
            }

            if (m_penalized) {
                if (!penalized) {
                    m_restored_from_penalized = false;
                    m_penalized = false;
                    m_goto->process(m_field->get_start_pose() - odo);
                    if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Robot is unpenalized";
                } else {
                    if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Robot is penalized";
                    return;
                }
            }

            if (!m_restored_from_penalized) {
                if (ball != cv::Rect()) {
                    m_restored_from_penalized = true;
                } else if (m_goto->is_done()) {
                    m_goto->process(m_field->get_start_pose() - odo);
                    return;
                }
            }

            // Switch to head and walking after action
            m_head->joint.set_enable_head_only(true, true);
            m_walking->joint.set_enable_body_without_head(true, true);
            m_tracker->process(ball_point);

            if (m_tracker->is_no_ball()) {
                eye_leds = color_t({255, 0, 0});
            }
            m_LEDs->set_eye_led(eye_leds);

            if (m_tracker->is_no_ball() && m_follower->is_no_ball()) {
                m_searcher->process();
                m_LEDs->set_head_led({0, 0, 255});
                return;
            } else {
                m_searcher->set_last_position(m_tracker->get_ball_position());
            }

            // Follow the ball
            m_follower->process(m_tracker->get_ball_position());
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
    if (m_avoid_localization) {
        if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Avoiding localization...";
        return;
    }

    if (m_rate_process_localization.is_passed() || m_force_localization) {
        if (m_debug) {
            if (m_force_localization) {
                LOG_DEBUG << "SOCCER BEHAVIOR: Forcing localization...";
            } else {
                LOG_DEBUG << "SOCCER BEHAVIOR: Processing localization...";
            }
        }

        const auto& odo = m_walking->get_odo();
        const auto& lines = m_vision->detect_lines();
        m_localization->set_pose_shift(odo);
        m_localization->set_lines(lines);
        m_localization->update();

        const auto& loc_pose_mean = m_localization->get_calculated_pose_mean();
        const auto& loc_pose_dev = m_localization->get_calculated_pose_std_dev();

        if (loc_pose_mean.is_nan() || loc_pose_dev.is_nan()) {
            if (m_debug) {
                LOG_DEBUG << "SOCCER BEHAVIOUR: NaNs detected in particle filter. Resetting...";
            }

            const particle_filter_t* pf = m_localization->get_particle_filter();
            pose2d_t last_pose = m_walking->get_odo();
            pose2d_t pose_dev(
                pf->get_loc_threshold_x(),
                pf->get_loc_threshold_y(),
                pf->get_loc_threshold_theta()
            );

            m_localization->set_pose_shift(last_pose);
            m_localization->reset_pose_approximate_area_around(last_pose, pose_dev);
        } else if (m_localization->is_localized() &&
                m_walking->get_a_move_amplitude() == 0.0f &&
                m_walking->get_y_move_amplitude() == 0.0f &&
                m_walking->get_x_move_amplitude() <= 4.0f
                ) {

            const auto& localized_pose = m_localization->get_calculated_pose_mean();

            if (m_debug) LOG_DEBUG << "SOCCER BEHAVIOR: Successfull localization to pose = " << localized_pose;

            m_walking->set_odo(localized_pose);
            // set_pose_shift to avoid big jumps
            m_localization->set_pose_shift(localized_pose);

            m_rate_process_localization.update();
            m_force_localization = false;
        }
    }
}
