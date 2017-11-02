/*!
 *  \autor arssivka
 *  \date 10/26/17
 */

#include "motion/modules/action_t.h"
#include "config/action_configuration_file_loader_t.h"

void drwn::action_configuration_file_loader_t::read_motion_file() {
    action_t::GetInstance()->load_file(m_path.c_str());
}

const std::string& drwn::action_configuration_file_loader_t::get_path() const {
    return m_path;
}

void drwn::action_configuration_file_loader_t::set_path(const std::string& path) {
    m_path = path;
}
