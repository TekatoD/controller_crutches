/*!
 *  \autor arssivka
 *  \date 10/26/17
 */

#include "motion/modules/action_t.h"
#include "config/ActionConfigurationFileLoader.h"

void drwn::ActionConfigurationFileLoader::ReadMotionFile() {
    action_t::GetInstance()->load_file(m_path.c_str());
}

const std::string& drwn::ActionConfigurationFileLoader::GetPath() const {
    return m_path;
}

void drwn::ActionConfigurationFileLoader::SetPath(const std::string& path) {
    m_path = path;
}
