/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/single_option_arguments_parsing_strategy_t.h"

using namespace drwn;

single_option_arguments_parsing_strategy_t::single_option_arguments_parsing_strategy_t(std::string option_name,
                                                                           std::string option_description)
        : m_option_name(move(option_name)),
          m_option_description(move(option_description)) {}

void single_option_arguments_parsing_strategy_t::set_option(std::string option_name, std::string option_description) {
    m_option_name = std::move(option_name);
    m_option_description = std::move(option_description);
}

const std::string& single_option_arguments_parsing_strategy_t::get_option_name() const noexcept {
    return m_option_name;
}

void single_option_arguments_parsing_strategy_t::set_option_name(std::string option_name) {
    m_option_name = std::move(option_name);
}

const std::string& single_option_arguments_parsing_strategy_t::get_option_description() const noexcept {
    return m_option_description;
}

void single_option_arguments_parsing_strategy_t::set_option_description(std::string option_description) {
    m_option_description = std::move(option_description);
}
