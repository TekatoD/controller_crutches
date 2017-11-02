/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/strategies/debug_mode_arguments_parsing_strategy_t.h"

using namespace drwn;

void debug_mode_arguments_parsing_strategy_t::define_description(boost::program_options::options_description& desc) {
    namespace po = boost::program_options;
    desc.add_options()
            (m_option_name.c_str(), po::value(&m_debug)->zero_tokens(), m_option_description.c_str());
}

void debug_mode_arguments_parsing_strategy_t::apply_variables(const boost::program_options::variables_map& vm) {
    // Do nothing
}

bool debug_mode_arguments_parsing_strategy_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void debug_mode_arguments_parsing_strategy_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

debug_mode_arguments_parsing_strategy_t::operator bool() const noexcept {
    return m_debug;
}

