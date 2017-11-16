/// \autor arssivka
/// \date 11/7/17

#include "config/strategies/robot_image_source_configuration_strategy_t.h"

using namespace drwn;

robot_image_source_configuration_strategy_t::robot_image_source_configuration_strategy_t(std::string section)
        : configuration_strategy_t(std::move(section)) {}

void robot_image_source_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (prop.count(this->get_section()) != 0) {
        auto& im_source_section = prop.get_child(this->get_section());

        auto hue = im_source_section.get_optional<float>("hue");
        auto brightness = im_source_section.get_optional<float>("brightness");
        auto contrast = im_source_section.get_optional<float>("contrast");
        auto gain = im_source_section.get_optional<float>("gain");
        auto width = im_source_section.get_optional<float>("width");
        auto height = im_source_section.get_optional<float>("height");

        if (hue) m_image_source->set_hue(hue.get());
        if (brightness) m_image_source->set_brightness(brightness.get());
        if (contrast) m_image_source->set_contrast(contrast.get());
        if (gain) m_image_source->set_gain(gain.get());
        if (width) m_image_source->set_width(width.get());
        if (height) m_image_source->set_height(height.get());
    }
}

void robot_image_source_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (prop.count(this->get_section()) == 0) prop.add_child(this->get_section(), {});
    auto& im_source_section = prop.get_child(this->get_section());

    im_source_section.put("hue", m_image_source->get_hue());
    im_source_section.put("brightness", m_image_source->get_brightness());
    im_source_section.put("contrast", m_image_source->get_contrast());
    im_source_section.put("gain", m_image_source->get_gain());
    im_source_section.put("width", m_image_source->get_width());
    im_source_section.put("height", m_image_source->get_height());
}

robot_image_source_t* robot_image_source_configuration_strategy_t::get_image_source() const noexcept {
    return m_image_source;
}

void robot_image_source_configuration_strategy_t::set_image_source(robot_image_source_t* image_source) noexcept {
    m_image_source = image_source;
}
