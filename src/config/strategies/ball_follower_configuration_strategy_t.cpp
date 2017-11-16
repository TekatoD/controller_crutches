/// \autor arssivka
/// \date 11/15/17

#include <behavior/ball_follower_t.h>
#include "config/strategies/ball_follower_configuration_strategy_t.h"

using namespace drwn;

void ball_follower_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (prop.count(this->get_section()) == 0) return; // Section doesn't exist
    
    auto& section = prop.get_child(this->get_section());

    auto follower = ball_follower_t::get_instance();
    auto no_ball_max_count = section.get_optional<int>("no_ball_max_count");
    if (no_ball_max_count) follower->set_no_ball_max_count(no_ball_max_count.get());
    auto kick_ball_max_count = section.get_optional<int>("kick_ball_max_count");
    if (kick_ball_max_count) follower->set_kick_ball_max_count(kick_ball_max_count.get());
    auto kick_top_angle = section.get_optional<float>("kick_top_angle");
    if (kick_top_angle) follower->set_kick_top_angle(kick_top_angle.get());
    auto kick_right_angle = section.get_optional<float>("kick_right_angle");
    if (kick_right_angle) follower->set_kick_right_angle(kick_right_angle.get());
    auto kick_left_angle = section.get_optional<float>("kick_left_angle");
    if (kick_left_angle) follower->set_kick_left_angle(kick_left_angle.get());
    auto follow_max_x_step = section.get_optional<float>("follow_max_x_step");
    if (follow_max_x_step) follower->set_follow_max_x_step(follow_max_x_step.get());
    auto follow_min_x_step = section.get_optional<float>("follow_min_x_step");
    if (follow_min_x_step) follower->set_follow_min_x_step(follow_min_x_step.get());
    auto follow_max_z_turn = section.get_optional<float>("follow_max_z_turn");
    if (follow_max_z_turn) follower->set_follow_max_z_turn(follow_max_z_turn.get());
    auto fit_x_step = section.get_optional<float>("fit_x_step");
    if (fit_x_step) follower->set_fit_x_step(fit_x_step.get());
    auto fit_max_z_turn = section.get_optional<float>("fit_max_z_turn");
    if (fit_max_z_turn) follower->set_fit_max_z_turn(fit_max_z_turn.get());
    auto unit_x_step = section.get_optional<float>("unit_x_step");
    if (unit_x_step) follower->set_unit_x_step(unit_x_step.get());
    auto unit_y_step = section.get_optional<float>("unit_y_step");
    if (unit_y_step) follower->set_unit_y_step(unit_y_step.get());
    auto unit_z_turn = section.get_optional<float>("unit_z_turn");
    if (unit_z_turn) follower->set_unit_z_turn(unit_z_turn.get());
    auto goal_x_step = section.get_optional<float>("goal_x_step");
    if (goal_x_step) follower->set_goal_x_step(goal_x_step.get());
    auto goal_y_step = section.get_optional<float>("goal_y_step");
    if (goal_y_step) follower->set_goal_y_step(goal_y_step.get());
    auto goal_z_turn = section.get_optional<float>("goal_z_turn");
    if (goal_z_turn) follower->set_goal_z_turn(goal_z_turn.get());
    auto x_step = section.get_optional<float>("x_step");
    if (x_step) follower->set_x_step(x_step.get());
    auto y_step = section.get_optional<float>("y_step");
    if (y_step) follower->set_y_step(y_step.get());
    auto z_turn = section.get_optional<float>("z_turn");
    if (z_turn) follower->set_z_turn(z_turn.get());
    auto tilt_offset = section.get_optional<float>("tilt_offset");
    if (tilt_offset) follower->set_tilt_offset(tilt_offset.get());
    auto aim_tilt_offset = section.get_optional<float>("aim_tilt_offset");
    if (aim_tilt_offset) follower->set_aim_tilt_offset(aim_tilt_offset.get());
    auto aim_y_turn = section.get_optional<float>("aim_y_turn");
    if (aim_y_turn) follower->set_aim_y_turn(aim_y_turn.get());
    auto aim_z_step = section.get_optional<float>("aim_z_step");
    if (aim_z_step) follower->set_aim_z_step(aim_z_step.get());
}

void ball_follower_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (prop.count(this->get_section()) == 0) prop.add_child(get_section(), {});

    auto& section = prop.get_child(this->get_section());
    auto follower = ball_follower_t::get_instance();
    section.put("no_ball_max_count", follower->get_no_ball_max_count());
    section.put("kick_ball_max_count", follower->get_kick_ball_max_count());
    section.put("kick_top_angle", follower->get_kick_top_angle());
    section.put("kick_right_angle", follower->get_kick_right_angle());
    section.put("kick_left_angle", follower->get_kick_left_angle());
    section.put("follow_max_x_step", follower->get_follow_max_x_step());
    section.put("follow_min_x_step", follower->get_follow_min_x_step());
    section.put("follow_max_z_turn", follower->get_follow_max_z_turn());
    section.put("fit_x_step", follower->get_fit_x_step());
    section.put("fit_max_z_turn", follower->get_fit_max_z_turn());
    section.put("unit_x_step", follower->get_unit_x_step());
    section.put("unit_y_step", follower->get_unit_y_step());
    section.put("unit_z_turn", follower->get_unit_z_turn());
    section.put("goal_x_step", follower->get_goal_x_step());
    section.put("goal_y_step", follower->get_goal_y_step());
    section.put("goal_z_turn", follower->get_goal_z_turn());
    section.put("x_step", follower->get_x_step());
    section.put("y_step", follower->get_y_step());
    section.put("z_turn", follower->get_z_turn());
    section.put("aim_y_turn", follower->get_aim_y_turn());
    section.put("get_aim_z_step", follower->get_aim_z_step());
}
