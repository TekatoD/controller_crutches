/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include "config/HeadConfigurationStrategy.h"
#include "motion/modules/Head.h"

using namespace Robot;

void HeadConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    Head* head = Head::GetInstance();

    auto pan_p_gain = prop.get_optional<float>("pan_p_gain");
    auto pan_d_gain = prop.get_optional<float>("pan_d_gain");
    auto tilt_p_gain = prop.get_optional<float>("tilt_p_gain");
    auto tilt_d_gain = prop.get_optional<float>("tilt_d_gain");
    auto left_limit = prop.get_optional<float>("left_limit");
    auto right_limit = prop.get_optional<float>("right_limit");
    auto top_limit = prop.get_optional<float>("top_limit");
    auto bottom_limit = prop.get_optional<float>("bottom_limit");
    auto pan_home = prop.get_optional<float>("pan_home");
    auto tilt_home = prop.get_optional<float>("tilt_home");

    if (pan_p_gain) head->SetPanPGain(pan_p_gain.get());
    if (pan_d_gain) head->SetPanDGain(pan_d_gain.get());
    if (tilt_p_gain) head->SetTiltPGain(tilt_p_gain.get());
    if (tilt_d_gain) head->SetTiltDGain(tilt_d_gain.get());
    if (left_limit) head->SetLeftLimit(left_limit.get());
    if (right_limit) head->SetRightLimit(right_limit.get());
    if (top_limit) head->SetTopLimit(top_limit.get());
    if (bottom_limit) head->SetBottomLimit(bottom_limit.get());
    if (pan_home ) head->SetPanHome(pan_home.get());
    if (tilt_home) head->SetTiltHome(tilt_home.get());
}

void HeadConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    Head* head = Head::GetInstance();

    prop.put("pan_p_gain", head->GetPanPGain());
    prop.put("pan_d_gain", head->GetPanDGain());
    prop.put("tilt_p_gain", head->GetTiltPGain());
    prop.put("tilt_d_gain", head->GetTiltDGain());
    prop.put("left_limit", head->GetLeftLimit());
    prop.put("right_limit", head->GetRightLimit());
    prop.put("top_limit", head->GetTopLimit());
    prop.put("bottom_limit", head->GetBottomLimit());
    prop.put("pan_home", head->GetPanHome());
    prop.put("tilt_home", head->GetTiltHome());
}