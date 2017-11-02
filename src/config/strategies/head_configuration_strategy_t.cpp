/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include "config/strategies/head_configuration_strategy_t.h"
#include "motion/modules/head_t.h"

using namespace drwn;

head_configuration_strategy_t::head_configuration_strategy_t(std::string section)
        : configuration_strategy_t(std::move(section)) { }

void head_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (prop.count(get_section()) == 0) return; // Section doesn't exist

    head_t* head = head_t::GetInstance();
    auto& head_section = prop.get_child(this->get_section());

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

    if (pan_p_gain) head->set_pan_p_gain(pan_p_gain.get());
    if (pan_d_gain) head->set_pan_d_gain(pan_d_gain.get());
    if (tilt_p_gain) head->set_tilt_p_gain(tilt_p_gain.get());
    if (tilt_d_gain) head->set_tilt_d_gain(tilt_d_gain.get());
    if (left_limit) head->set_left_limit(left_limit.get());
    if (right_limit) head->set_right_limit(right_limit.get());
    if (top_limit) head->set_top_limit(top_limit.get());
    if (bottom_limit) head->set_bottom_limit(bottom_limit.get());
    if (pan_home ) head->set_pan_home(pan_home.get());
    if (tilt_home) head->set_tilt_home(tilt_home.get());
}

void head_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (prop.count(get_section()) == 0) prop.add_child(get_section(), {});

    head_t* head = head_t::GetInstance();
    auto& head_section = prop.get_child(this->get_section());

    head_section.put("pan_p_gain", head->get_pan_p_gain());
    head_section.put("pan_d_gain", head->get_pan_d_gain());
    head_section.put("tilt_p_gain", head->get_tilt_p_gain());
    head_section.put("tilt_d_gain", head->get_tilt_d_gain());
    head_section.put("left_limit", head->get_left_limit());
    head_section.put("right_limit", head->get_right_limit());
    head_section.put("top_limit", head->get_top_limit());
    head_section.put("bottom_limit", head->get_bottom_limit());
    head_section.put("pan_home", head->get_pan_home());
    head_section.put("tilt_home", head->get_tilt_home());
}
