/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/arguments_parser_t.h"

using namespace drwn;

arguments_parser_t::arguments_parser_t(std::string caption)
        : m_caption(std::move(caption)) {}

void arguments_parser_t::add_strategy(arguments_parsing_strategy_t& strategy) {
    m_strategies.push_back(&strategy);
}

void arguments_parser_t::remove_strategy(const arguments_parsing_strategy_t& strategy) {
    m_strategies.remove((arguments_parsing_strategy_t*) &strategy);
}

bool arguments_parser_t::parse() {
    namespace po = boost::program_options;
    try {
        auto desk = generate_option_description();
        po::variables_map vm;
        po::store(po::command_line_parser(m_arguments.get_argc(), m_arguments.get_argv())
                          .options(desk)
                          .allow_unregistered()
                          .run(), vm);
        po::notify(vm);
        apply_variables_map(vm);
    } catch (const boost::program_options::error& e) {
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}

boost::program_options::options_description arguments_parser_t::generate_option_description() const {
    boost::program_options::options_description desk(m_caption);
    for (auto& strategy : m_strategies) {
        strategy->define_description(desk);
    }
    return desk;
}

void arguments_parser_t::apply_variables_map(const boost::program_options::variables_map& vm) {
    for (auto& strategy : m_strategies) {
        strategy->apply_variables(vm);
    }
}

const std::string& arguments_parser_t::get_caption() const noexcept {
    return m_caption;
}

void arguments_parser_t::set_caption(const std::string& caption) {
    m_caption = caption;
}

void arguments_parser_t::show_description_to_stream(std::ostream& stream) const {
    stream << generate_option_description();
}

const command_arguments_t& arguments_parser_t::get_arguments() const noexcept {
    return m_arguments;
}

void arguments_parser_t::set_arguments(const command_arguments_t& arguments) noexcept {
    m_arguments = arguments;
}
