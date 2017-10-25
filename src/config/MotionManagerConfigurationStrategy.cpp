/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/Walking.h>
#include <motion/MotionManager.h>
#include <iomanip>
#include <sstream>
#include "config/MotionManagerConfigurationStrategy.h"

using namespace Robot;

void MotionManagerConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    MotionManager* motionManager = MotionManager::GetInstance();

    for (int i = 1; i < JointData::NUMBER_OF_JOINTS; ++i) {
        std::stringstream keyStream;
        keyStream << "ID_" << std::setw(2) << std::setfill('0') << i;
        auto offset = prop.get_optional<int>(keyStream.str());
        if (offset) motionManager->SetJointOffset(i, offset.get());
    }
}

void MotionManagerConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    MotionManager* motionManager = MotionManager::GetInstance();

    for (int i = 1; i < JointData::NUMBER_OF_JOINTS; ++i) {
        std::stringstream keyStream;
        keyStream << "ID_" << std::setw(2) << std::setfill('0') << i;
        prop.put(keyStream.str(), motionManager->GetJointOffset(i));
    }
}