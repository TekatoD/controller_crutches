/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/strategies/DebugModeArgumentsParsingStrategy.h"

using namespace Robot;

void DebugModeArgumentsParsingStrategy::DefineDescription(boost::program_options::options_description& desc) {
    namespace po = boost::program_options;
    desc.add_options()
            (m_option_name.c_str(), po::value(&m_debug)->zero_tokens(), m_option_description.c_str());
}

void DebugModeArgumentsParsingStrategy::ApplyVariables(const boost::program_options::variables_map& vm) {
    // Do nothing
}

bool DebugModeArgumentsParsingStrategy::IsDebugEnabled() const noexcept {
    return m_debug;
}

void DebugModeArgumentsParsingStrategy::EnableDebug(bool debug) noexcept {
    m_debug = debug;
}

DebugModeArgumentsParsingStrategy::operator bool() const noexcept {
    return m_debug;
}

