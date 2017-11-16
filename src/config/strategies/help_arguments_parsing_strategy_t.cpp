/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/strategies/help_arguments_parsing_strategy_t.h"

drwn::help_arguments_parsing_strategy_t::help_arguments_parsing_strategy_t()
        : single_option_arguments_parsing_strategy_t("h,help", "produce help message") {}

void drwn::help_arguments_parsing_strategy_t::define_description(boost::program_options::options_description& desc) {
    namespace po = boost::program_options;
    desc.add_options()
            (m_option_name.c_str(), po::value(&m_help_requested)->zero_tokens(), m_option_description.c_str());
}

void drwn::help_arguments_parsing_strategy_t::apply_variables(const boost::program_options::variables_map& vm) {
    // Do nothing
}

bool drwn::help_arguments_parsing_strategy_t::is_help_requested() const {
    return m_help_requested;
}

void drwn::help_arguments_parsing_strategy_t::set_help_requested(bool help_requested) {
    m_help_requested = help_requested;
}
