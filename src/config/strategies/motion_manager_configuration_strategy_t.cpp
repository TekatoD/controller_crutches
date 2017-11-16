/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/walking_t.h>
#include <motion/motion_manager_t.h>
#include <iomanip>
#include <sstream>
#include "config/strategies/motion_manager_configuration_strategy_t.h"

using namespace drwn;

motion_manager_configuration_strategy_t::motion_manager_configuration_strategy_t(std::string section)
        : configuration_strategy_t(std::move(section)) { }

void motion_manager_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    motion_manager_t* motion_manager = motion_manager_t::get_instance();
    if (prop.count(this->get_section()) == 0) return; // Section doesn't exist

    auto& motion_manager_section = prop.get_child(get_section());
    for (int i = 1; i < joint_data_t::NUMBER_OF_JOINTS; ++i) {
        std::stringstream key_stream;
        key_stream << "ID_" << std::setw(2) << std::setfill('0') << i;
        auto offset = motion_manager_section.get_optional<int>(key_stream.str());
        if (offset) motion_manager->set_joint_offset(i, offset.get());
    }
}

void motion_manager_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    motion_manager_t* motion_manager = motion_manager_t::get_instance();
    if (prop.count(get_section()) == 0) prop.add_child(get_section(), {});

    auto& motion_manager_section = prop.get_child(this->get_section());
    for (int i = 1; i < joint_data_t::NUMBER_OF_JOINTS; ++i) {
        std::stringstream key_stream;
        key_stream << "ID_" << std::setw(2) << std::setfill('0') << i;
        motion_manager_section.put(key_stream.str(), motion_manager->get_joint_offset(i));
    }
}
