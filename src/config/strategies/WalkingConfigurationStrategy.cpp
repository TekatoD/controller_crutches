#include <motion/modules/walking_t.h>
#include "config/strategies/WalkingConfigurationStrategy.h"

using namespace drwn;

WalkingConfigurationStrategy::WalkingConfigurationStrategy(std::string section)
        : ConfigurationStrategy(std::move(section)) { }

void WalkingConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    if (prop.count(GetSection()) == 0) return; // Section doesn't exist

    walking_t* walking = walking_t::GetInstance();
    auto& walking_section = prop.get_child(this->GetSection());

    auto odo_x_factor = walking_section.get_optional<float>("odo_x_factor");
    auto odo_y_factor = walking_section.get_optional<float>("odo_y_factor");
    auto odo_a_factor = walking_section.get_optional<float>("odo_a_factor");
    auto x_offset = walking_section.get_optional<float>("x_offset");
    auto y_offset = walking_section.get_optional<float>("y_offset");
    auto z_offset = walking_section.get_optional<float>("z_offset");
    auto roll_offset = walking_section.get_optional<float>("roll_offset");
    auto pitch_offset = walking_section.get_optional<float>("pitch_offset");
    auto yaw_offset = walking_section.get_optional<float>("yaw_offset");
    auto hip_pitch_offset = walking_section.get_optional<float>("hip_pitch_offset");
    auto period_time = walking_section.get_optional<float>("period_time");
    auto dsp_ratio = walking_section.get_optional<float>("dsp_ratio");
    auto step_forward_back_ratio = walking_section.get_optional<float>("step_forward_back_ratio");
    auto foot_height = walking_section.get_optional<float>("foot_height");
    auto swing_right_left = walking_section.get_optional<float>("swing_right_left");
    auto swing_top_down = walking_section.get_optional<float>("swing_top_down");
    auto pelvis_offset = walking_section.get_optional<float>("pelvis_offset");
    auto arm_swing_gain = walking_section.get_optional<float>("arm_swing_gain");
    auto balance_knee_gain = walking_section.get_optional<float>("balance_knee_gain");
    auto balance_ankle_pitch_gain = walking_section.get_optional<float>("balance_ankle_pitch_gain");
    auto balance_hip_roll_gain = walking_section.get_optional<float>("balance_hip_roll_gain");
    auto balance_ankle_roll_gain = walking_section.get_optional<float>("balance_ankle_roll_gain");

    auto p_gain = walking_section.get_optional<int>("p_gain");
    auto i_gain = walking_section.get_optional<int>("i_gain");
    auto d_gain = walking_section.get_optional<int>("d_gain");

    if (odo_x_factor) walking->set_odo_x_factor(odo_x_factor.get());
    if (odo_y_factor) walking->set_odo_y_factor(odo_y_factor.get());
    if (odo_a_factor) walking->set_odo_a_factor(odo_a_factor.get());
    if (x_offset) walking->set_x_offset(x_offset.get());
    if (y_offset) walking->set_y_offset(y_offset.get());
    if (z_offset) walking->set_z_offset(z_offset.get());
    if (roll_offset) walking->set_roll_offset(roll_offset.get());
    if (pitch_offset) walking->set_pitch_offset(pitch_offset.get());
    if (yaw_offset) walking->set_yaw_offset(yaw_offset.get());
    if (hip_pitch_offset) walking->set_hip_pitch_offset(hip_pitch_offset.get());
    if (period_time) walking->set_period_time(period_time.get());
    if (dsp_ratio) walking->set_DSP_ratio(dsp_ratio.get());
    if (step_forward_back_ratio) walking->set_step_FB_ratio(step_forward_back_ratio.get());
    if (foot_height) walking->set_z_move_amplitude(foot_height.get());
    if (swing_right_left) walking->set_y_swap_amplitude(swing_right_left.get());
    if (swing_top_down) walking->set_z_swap_amplitude(swing_top_down.get());
    if (pelvis_offset) walking->set_pelvis_offset(pelvis_offset.get());
    if (arm_swing_gain) walking->set_arm_swing_gain(arm_swing_gain.get());
    if (balance_knee_gain) walking->set_balance_knee_gain(balance_knee_gain.get());
    if (balance_ankle_pitch_gain) walking->set_balance_ankle_pitch_gain(balance_ankle_pitch_gain.get());
    if (balance_hip_roll_gain) walking->set_balance_hip_roll_gain(balance_hip_roll_gain.get());
    if (balance_ankle_roll_gain) walking->set_balance_ankle_roll_gain(balance_ankle_roll_gain.get());

    if (p_gain) walking->set_p_gain(p_gain.get());
    if (i_gain) walking->set_i_gain(i_gain.get());
    if (d_gain) walking->set_d_gain(d_gain.get());
}

void WalkingConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    if (prop.count(GetSection()) == 0) prop.add_child(GetSection(), {});

    walking_t* walking = walking_t::GetInstance();
    auto& walking_section = prop.get_child(this->GetSection());

    walking_section.put("odo_x_factor", walking->get_odo_x_factor());
    walking_section.put("odo_y_factor", walking->get_odo_y_factor());
    walking_section.put("odo_a_factor", walking->get_odo_a_factor());

    walking_section.put("x_offset", walking->get_x_offset());
    walking_section.put("y_offset", walking->get_y_offset());
    walking_section.put("z_offset", walking->get_z_offset());
    walking_section.put("roll_offset", walking->get_roll_offset());
    walking_section.put("pitch_offset", walking->get_pitch_offset());
    walking_section.put("yaw_offset", walking->get_yaw_offset());
    walking_section.put("hip_pitch_offset", walking->get_hip_pitch_offset());
    walking_section.put("period_time", walking->get_period_time());
    walking_section.put("dsp_ratio", walking->get_DSP_ratio());
    walking_section.put("step_forward_back_ratio", walking->get_step_FB_ratio());
    walking_section.put("foot_height", walking->get_z_move_amplitude());
    walking_section.put("swing_right_left", walking->get_y_swap_amplitude());
    walking_section.put("swing_top_down", walking->get_z_swap_amplitude());
    walking_section.put("pelvis_offset", walking->get_pelvis_offset());
    walking_section.put("arm_swing_gain", walking->get_arm_swing_gain());
    walking_section.put("balance_knee_gain", walking->get_balance_knee_gain());
    walking_section.put("balance_ankle_pitch_gain", walking->get_balance_ankle_pitch_gain());
    walking_section.put("balance_hip_roll_gain", walking->get_balance_hip_roll_gain());
    walking_section.put("balance_ankle_roll_gain", walking->get_balance_ankle_roll_gain());

    walking_section.put("p_gain", walking->get_p_gain());
    walking_section.put("i_gain", walking->get_i_gain());
    walking_section.put("d_gain", walking->get_d_gain());

}
