/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/SingleOptionArgumentsParsingStrategy.h"

using namespace Robot;

SingleOptionArgumentsParsingStrategy::SingleOptionArgumentsParsingStrategy(std::string option_name,
                                                                           std::string option_description)
        : m_option_name(move(option_name)),
          m_option_description(move(option_description)) {}

void SingleOptionArgumentsParsingStrategy::SetOption(std::string option_name, std::string option_description) {
    m_option_name = std::move(option_name);
    m_option_description = std::move(option_description);
}

const std::string& SingleOptionArgumentsParsingStrategy::GetOptionName() const noexcept {
    return m_option_name;
}

void SingleOptionArgumentsParsingStrategy::SetOptionName(std::string option_name) {
    m_option_name = std::move(option_name);
}

const std::string& SingleOptionArgumentsParsingStrategy::GetOptionDescription() const noexcept {
    return m_option_description;
}

void SingleOptionArgumentsParsingStrategy::SetOptionDescription(std::string option_description) {
    m_option_description = std::move(option_description);
}
