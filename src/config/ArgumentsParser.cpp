/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#include "config/ArgumentsParser.h"

using namespace Robot;

ArgumentsParser::ArgumentsParser(std::string caption)
        : m_caption(std::move(caption)) {}

void ArgumentsParser::AddStrategy(ArgumentsParsingStrategy& strategy) {
    m_strategies.push_back(&strategy);
}

void ArgumentsParser::RemoveStrategy(const ArgumentsParsingStrategy& strategy) {
    m_strategies.remove((ArgumentsParsingStrategy*) &strategy);
}

bool ArgumentsParser::Parse() {
    namespace po = boost::program_options;
    try {
        auto desk = GenerateOptionDescription();
        po::variables_map vm;
        po::store(po::command_line_parser(m_arguments.GetArgc(), m_arguments.GetArgv())
                          .options(desk)
                          .allow_unregistered()
                          .run(), vm);
        po::notify(vm);
        ApplyVariablesMap(vm);
    } catch (const boost::program_options::error& e) {
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}

boost::program_options::options_description ArgumentsParser::GenerateOptionDescription() const {
    boost::program_options::options_description desk(m_caption);
    for (auto& strategy : m_strategies) {
        strategy->DefineDescription(desk);
    }
    return desk;
}

void ArgumentsParser::ApplyVariablesMap(const boost::program_options::variables_map& vm) {
    for (auto& strategy : m_strategies) {
        strategy->ApplyVariables(vm);
    }
}

const std::string& ArgumentsParser::GetCaption() const noexcept {
    return m_caption;
}

void ArgumentsParser::SetCaption(const std::string& caption) {
    m_caption = caption;
}

void ArgumentsParser::ShowDescriptionToStream(std::ostream& stream) const {
    stream << GenerateOptionDescription();
}

const CommandArguments& ArgumentsParser::GetArguments() const noexcept {
    return m_arguments;
}

void ArgumentsParser::SetArguments(const CommandArguments& arguments) noexcept {
    m_arguments = arguments;
}
