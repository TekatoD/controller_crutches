/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/strategies/HelpArgumentsParsingStrategy.h"

drwn::HelpArgumentsParsingStrategy::HelpArgumentsParsingStrategy()
        : SingleOptionArgumentsParsingStrategy("h,help", "produce help message") {}

void drwn::HelpArgumentsParsingStrategy::DefineDescription(boost::program_options::options_description& desc) {
    namespace po = boost::program_options;
    desc.add_options()
            (m_option_name.c_str(), po::value(&m_help_requested)->zero_tokens(), m_option_description.c_str());
}

void drwn::HelpArgumentsParsingStrategy::ApplyVariables(const boost::program_options::variables_map& vm) {
    // Do nothing
}

bool drwn::HelpArgumentsParsingStrategy::IsHelpRequested() const {
    return m_help_requested;
}

void drwn::HelpArgumentsParsingStrategy::SetHelpRequested(bool help_requested) {
    m_help_requested = help_requested;
}
