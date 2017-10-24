#include <motion/modules/Walking.h>
#include "config/WalkingConfigurationStrategy.h"

using namespace Robot;

void WalkingConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    Walking* walking = Walking::GetInstance();

    auto odo_x_factor = prop.get_optional<float>("odo_x_factor");
    auto odo_y_factor = prop.get_optional<float>("odo_y_factor");
    auto odo_a_factor = prop.get_optional<float>("odo_a_factor");
    auto x_offset = prop.get_optional<float>("x_offset");
    auto y_offset = prop.get_optional<float>("y_offset");
    auto z_offset = prop.get_optional<float>("z_offset");
    auto roll_offset = prop.get_optional<float>("roll_offset");
    auto pitch_offset = prop.get_optional<float>("pitch_offset");
    auto yaw_offset = prop.get_optional<float>("yaw_offset");
    auto hip_pitch_offset = prop.get_optional<float>("hip_pitch_offset");
    auto period_time = prop.get_optional<float>("period_time");
    auto dsp_ratio = prop.get_optional<float>("dsp_ratio");
    auto step_forward_back_ratio = prop.get_optional<float>("step_forward_back_ratio");
    auto foot_height = prop.get_optional<float>("foot_height");
    auto swing_right_left = prop.get_optional<float>("swing_right_left");
    auto swing_top_down = prop.get_optional<float>("swing_top_down");
    auto pelvis_offset = prop.get_optional<float>("pelvis_offset");
    auto arm_swing_gain = prop.get_optional<float>("arm_swing_gain");
    auto balance_knee_gain = prop.get_optional<float>("balance_knee_gain");
    auto balance_ankle_pitch_gain = prop.get_optional<float>("balance_ankle_pitch_gain");
    auto balance_hip_roll_gain = prop.get_optional<float>("balance_hip_roll_gain");
    auto balance_ankle_roll_gain = prop.get_optional<float>("balance_ankle_roll_gain");

    auto p_gain = prop.get_optional<int>("p_gain");
    auto i_gain = prop.get_optional<int>("i_gain");
    auto d_gain = prop.get_optional<int>("d_gain");

    if (odo_x_factor) walking->SetOdoXFactor(odo_x_factor.get());
    if (odo_y_factor) walking->SetOdoYFactor(odo_y_factor.get());
    if (odo_a_factor) walking->SetOdoAFactor(odo_a_factor.get());
    if (x_offset) walking->SetXOffset(x_offset.get());
    if (y_offset) walking->SetYOffset(y_offset.get());
    if (z_offset) walking->SetZOffset(z_offset.get());
    if (roll_offset) walking->SetRollOffset(roll_offset.get());
    if (pitch_offset) walking->SetPitchOffset(pitch_offset.get());
    if (yaw_offset) walking->SetYawOffset(yaw_offset.get());
    if (hip_pitch_offset) walking->SetHipPitchOffset(hip_pitch_offset.get());
    if (period_time) walking->SetPeriodTime(period_time.get());
    if (dsp_ratio) walking->SetDSPRatio(dsp_ratio.get());
    if (step_forward_back_ratio) walking->SetStepFBRatio(step_forward_back_ratio.get());
    if (foot_height) walking->SetZMoveAmplitude(foot_height.get());
    if (swing_right_left) walking->SetYSwapAmplitude(swing_right_left.get());
    if (swing_top_down) walking->SetZSwapAmplitude(swing_top_down.get());
    if (pelvis_offset) walking->SetPelvisOffset(pelvis_offset.get());
    if (arm_swing_gain) walking->SetArmSwingGain(arm_swing_gain.get());
    if (balance_knee_gain) walking->SetBalanceKneeGain(balance_knee_gain.get());
    if (balance_ankle_pitch_gain) walking->SetBalanceAnklePitchGain(balance_ankle_pitch_gain.get());
    if (balance_hip_roll_gain) walking->SetBalanceHipRollGain(balance_hip_roll_gain.get());
    if (balance_ankle_roll_gain) walking->SetBalanceAnkleRollGain(balance_ankle_roll_gain.get());

    if (p_gain) walking->SetPGain(p_gain.get());
    if (i_gain) walking->SetIGain(i_gain.get());
    if (d_gain) walking->SetDGain(d_gain.get());
}

void WalkingConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    Walking* walking = Walking::GetInstance();

    prop.put("odo_x_factor", walking->GetOdoXFactor());
    prop.put("odo_y_factor", walking->GetOdoYFactor());
    prop.put("odo_a_factor", walking->GetOdoAFactor());

    prop.put("x_offset", walking->GetXOffset());
    prop.put("y_offset", walking->GetYOffset());
    prop.put("z_offset", walking->GetZOffset());
    prop.put("roll_offset", walking->GetRollOffset());
    prop.put("pitch_offset", walking->GetPitchOffset());
    prop.put("yaw_offset", walking->GetYawOffset());
    prop.put("hip_pitch_offset", walking->GetHipPitchOffset());
    prop.put("period_time", walking->GetPeriodTime());
    prop.put("dsp_ratio", walking->GetDSPRatio());
    prop.put("step_forward_back_ratio", walking->GetStepFBRatio());
    prop.put("foot_height", walking->GetZMoveAmplitude());
    prop.put("swing_right_left", walking->GetYSwapAmplitude());
    prop.put("swing_top_down", walking->GetZSwapAmplitude());
    prop.put("pelvis_offset", walking->GetPelvisOffset());
    prop.put("arm_swing_gain", walking->GetArmSwingGain());
    prop.put("balance_knee_gain", walking->GetBalanceKneeGain());
    prop.put("balance_ankle_pitch_gain", walking->GetBalanceAnklePitchGain());
    prop.put("balance_hip_roll_gain", walking->GetBalanceHipRollGain());
    prop.put("balance_ankle_roll_gain", walking->GetBalanceAnkleRollGain());

    prop.put("p_gain", walking->GetPGain());
    prop.put("i_gain", walking->GetIGain());
    prop.put("d_gain", walking->GetDGain());

}
