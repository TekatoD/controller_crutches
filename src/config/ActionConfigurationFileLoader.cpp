/*!
 *  \autor arssivka
 *  \date 10/26/17
 */

#include "motion/modules/Action.h"
#include "config/ActionConfigurationFileLoader.h"

void Robot::ActionConfigurationFileLoader::ReadMotionFile() {
    Action::GetInstance()->LoadFile(m_path.c_str());
}

const std::string& Robot::ActionConfigurationFileLoader::GetPath() const {
    return m_path;
}

void Robot::ActionConfigurationFileLoader::SetPath(const std::string& path) {
    m_path = path;
}
