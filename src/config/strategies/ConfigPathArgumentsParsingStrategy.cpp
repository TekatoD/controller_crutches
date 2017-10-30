/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/strategies/ConfigPathArgumentsParsingStrategy.h"

using namespace Robot;

void ConfigPathArgumentsParsingStrategy::DefineDescription(boost::program_options::options_description& desc) {
    namespace po = boost::program_options;
    auto& option_name = GetOptionName();
    auto& option_description = GetOptionDescription();
    desc.add_options()
            (option_name.c_str(), po::value(&m_path)->value_name("path"), option_description.c_str());
}

void ConfigPathArgumentsParsingStrategy::ApplyVariables(const boost::program_options::variables_map& vm) {
    // Do nothing
}

const std::string& ConfigPathArgumentsParsingStrategy::GetPath() const {
    return m_path;
}

void ConfigPathArgumentsParsingStrategy::SetPath(const std::string& path) {
    m_path = path;
}

ConfigPathArgumentsParsingStrategy::operator const std::string&() const noexcept {
    return m_path;
}
