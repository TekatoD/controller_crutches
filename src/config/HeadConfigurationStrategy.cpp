/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include "config/HeadConfigurationStrategy.h"
#include "motion/modules/Head.h"

using namespace Robot;

HeadConfigurationStrategy::HeadConfigurationStrategy(std::string section)
        : ConfigurationStrategy(std::move(section)) { }

void HeadConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    if (prop.count(GetSection()) == 0) return; // Section doesn't exist

    Head* head = Head::GetInstance();
    auto& head_section = prop.get_child(this->GetSection());

    auto pan_p_gain = head_section.get_optional<float>("pan_p_gain");
    auto pan_d_gain = head_section.get_optional<float>("pan_d_gain");
    auto tilt_p_gain = head_section.get_optional<float>("tilt_p_gain");
    auto tilt_d_gain = head_section.get_optional<float>("tilt_d_gain");
    auto left_limit = head_section.get_optional<float>("left_limit");
    auto right_limit = head_section.get_optional<float>("right_limit");
    auto top_limit = head_section.get_optional<float>("top_limit");
    auto bottom_limit = head_section.get_optional<float>("bottom_limit");
    auto pan_home = head_section.get_optional<float>("pan_home");
    auto tilt_home = head_section.get_optional<float>("tilt_home");

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
    if (prop.count(GetSection()) == 0) prop.add_child(GetSection(), {});

    Head* head = Head::GetInstance();
    auto& head_section = prop.get_child(this->GetSection());

    head_section.put("pan_p_gain", head->GetPanPGain());
    head_section.put("pan_d_gain", head->GetPanDGain());
    head_section.put("tilt_p_gain", head->GetTiltPGain());
    head_section.put("tilt_d_gain", head->GetTiltDGain());
    head_section.put("left_limit", head->GetLeftLimit());
    head_section.put("right_limit", head->GetRightLimit());
    head_section.put("top_limit", head->GetTopLimit());
    head_section.put("bottom_limit", head->GetBottomLimit());
    head_section.put("pan_home", head->GetPanHome());
    head_section.put("tilt_home", head->GetTiltHome());
}
