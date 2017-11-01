/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/walking_t.h>
#include <motion/motion_manager_t.h>
#include <iomanip>
#include <sstream>
#include "config/strategies/MotionManagerConfigurationStrategy.h"

using namespace drwn;

MotionManagerConfigurationStrategy::MotionManagerConfigurationStrategy(std::string section)
        : ConfigurationStrategy(std::move(section)) { }

void MotionManagerConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    motion_manager_t* motionManager = motion_manager_t::GetInstance();
    if (prop.count(GetSection()) == 0) return; // Section doesn't exist

    auto& motion_manager_section = prop.get_child(GetSection());
    for (int i = 1; i < joint_data_t::NUMBER_OF_JOINTS; ++i) {
        std::stringstream keyStream;
        keyStream << "ID_" << std::setw(2) << std::setfill('0') << i;
        auto offset = motion_manager_section.get_optional<int>(keyStream.str());
        if (offset) motionManager->set_joint_offset(i, offset.get());
    }
}

void MotionManagerConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    motion_manager_t* motionManager = motion_manager_t::GetInstance();
    if (prop.count(GetSection()) == 0) prop.add_child(GetSection(), {});

    auto& motion_manager_section = prop.get_child(GetSection());
    for (int i = 1; i < joint_data_t::NUMBER_OF_JOINTS; ++i) {
        std::stringstream keyStream;
        keyStream << "ID_" << std::setw(2) << std::setfill('0') << i;
        motion_manager_section.put(keyStream.str(), motionManager->get_joint_offset(i));
    }
}
