/// \autor arssivka
/// \date 11/15/17

#include <behavior/ball_follower_t.h>
#include "config/strategies/ball_follower_configuration_strategy_t.h"

using namespace drwn;

void ball_follower_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (prop.count(this->get_section()) == 0) return; // Section doesn't exist
    
    auto& section = prop.get_child(this->get_section());

    auto follower = ball_follower_t::get_instance();

    auto kick_ball_rate = section.get_optional<float>("kick_ball_rate_ms");
    if (kick_ball_rate) follower->set_kick_ball_rate(std::chrono::milliseconds(kick_ball_rate.get()));
    auto no_ball_rate = section.get_optional<float>("no_ball_rate_ms");
    if (no_ball_rate) follower->set_no_ball_rate(std::chrono::milliseconds(no_ball_rate.get()));
    auto slanting_kick_angle = section.get_optional<float>("slanting_kick_angle");
    if (slanting_kick_angle) follower->set_slanting_kick_angle(slanting_kick_angle.get());
    auto straight_kick_angle = section.get_optional<float>("straight_kick_angle");
    if (straight_kick_angle) follower->set_straight_kick_angle(straight_kick_angle.get());
    auto follow_max_x_amplitude = section.get_optional<float>("follow_max_x_amplitude");
    if (follow_max_x_amplitude) follower->set_follow_max_x_amplitude(follow_max_x_amplitude.get());
    auto follow_min_x_amplitude = section.get_optional<float>("follow_min_x_amplitude");
    if (follow_min_x_amplitude) follower->set_follow_min_x_amplitude(follow_min_x_amplitude.get());
    auto follow_max_a_amplitude = section.get_optional<float>("follow_max_a_amplitude");
    if (follow_max_a_amplitude) follower->set_follow_max_a_amplitude(follow_max_a_amplitude.get());
    auto fit_x_amplitude = section.get_optional<float>("fit_x_amplitude");
    if (fit_x_amplitude) follower->set_fit_x_amplitude(fit_x_amplitude.get());
    auto fit_a_amplitude = section.get_optional<float>("fit_a_amplitude");
    if (fit_a_amplitude) follower->set_fit_a_amplitude(fit_a_amplitude.get());
    auto x_accel_step = section.get_optional<float>("x_accel_step");
    if (x_accel_step) follower->set_x_accel_step(x_accel_step.get());
    auto y_accel_step = section.get_optional<float>("y_accel_step");
    if (y_accel_step) follower->set_y_accel_step(y_accel_step.get());
    auto a_accel_step = section.get_optional<float>("a_accel_step");
    if (a_accel_step) follower->set_a_accel_step(a_accel_step.get());
    auto kick_tilt_offset = section.get_optional<float>("kick_tilt_offset");
    if (kick_tilt_offset) follower->set_kick_tilt_offset(kick_tilt_offset.get());
    auto fit_tilt_offset = section.get_optional<float>("fit_tilt_offset");
    if (fit_tilt_offset) follower->set_fit_tilt_offset(fit_tilt_offset.get());
    
}

void ball_follower_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    using namespace std::chrono;
    if (prop.count(this->get_section()) == 0) prop.add_child(get_section(), {});
    auto& section = prop.get_child(this->get_section());
    auto follower = ball_follower_t::get_instance();

    section.put("kick_ball_rate_ms", duration_cast<milliseconds>(follower->get_kick_ball_rate()).count());
    section.put("no_ball_rate_ms", duration_cast<milliseconds>(follower->get_no_ball_rate()).count());
    section.put("slanting_kick_angle", follower->get_slanting_kick_angle());
    section.put("straight_kick_angle", follower->get_straight_kick_angle());
    section.put("follow_max_x_amplitude", follower->get_follow_max_x_amplitude());
    section.put("follow_min_x_amplitude", follower->get_follow_min_x_amplitude());
    section.put("follow_max_a_amplitude", follower->get_follow_max_a_amplitude());
    section.put("fit_x_amplitude", follower->get_fit_x_amplitude());
    section.put("fit_a_amplitude", follower->get_fit_a_amplitude());
    section.put("x_accel_step", follower->get_x_accel_step());
    section.put("y_accel_step", follower->get_y_accel_step());
    section.put("a_accel_step", follower->get_a_accel_step());
    section.put("kick_tilt_offset", follower->get_kick_tilt_offset());
    section.put("fit_tilt_offset", follower->get_fit_tilt_offset());
}
