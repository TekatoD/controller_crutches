/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#pragma once


#include <list>
#include "ArgumentsParsingStrategy.h"
#include "CommandArguments.h"

namespace Robot {
    class ArgumentsParser {
    public:
        ArgumentsParser(std::string caption = "");

        void AddStrategy(ArgumentsParsingStrategy& strategy);

        void RemoveStrategy(const ArgumentsParsingStrategy& strategy);

        bool Parse();

        const std::string& GetCaption() const noexcept;

        void SetCaption(const std::string& caption);

        void ShowDescriptionToStream(std::ostream& stream) const;

        const CommandArguments& GetArguments() const noexcept;

        void SetArguments(const CommandArguments& arguments) noexcept;

    private:
        boost::program_options::options_description GenerateOptionDescription() const;

    private:
        using StrategiesList = std::list<ArgumentsParsingStrategy*>;

        StrategiesList m_strategies;
        CommandArguments m_arguments;
        std::string m_caption;

        void ApplyVariablesMap(const boost::program_options::variables_map& vm);
    };
}
