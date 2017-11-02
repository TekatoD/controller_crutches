/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/strategies/config_path_arguments_parsing_strategy_t.h"

using namespace drwn;

void config_path_arguments_parsing_strategy_t::define_description(boost::program_options::options_description& desc) {
    namespace po = boost::program_options;
    auto& option_name = get_option_name();
    auto& option_description = get_option_description();
    desc.add_options()
            (option_name.c_str(), po::value(&m_path)->value_name("path"), option_description.c_str());
}

void config_path_arguments_parsing_strategy_t::apply_variables(const boost::program_options::variables_map& vm) {
    // Do nothing
}

const std::string& config_path_arguments_parsing_strategy_t::get_path() const {
    return m_path;
}

void config_path_arguments_parsing_strategy_t::set_path(const std::string& path) {
    m_path = path;
}

config_path_arguments_parsing_strategy_t::operator const std::string&() const noexcept {
    return m_path;
}
