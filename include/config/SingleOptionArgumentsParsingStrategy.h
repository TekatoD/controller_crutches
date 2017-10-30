/*!
 *  \autor arssivka
 *  \date 10/27/17
 */
#pragma once


#include <boost/program_options.hpp>
#include "config/ArgumentsParsingStrategy.h"

namespace Robot {
    class SingleOptionArgumentsParsingStrategy
            : public ArgumentsParsingStrategy {
    public:
        explicit SingleOptionArgumentsParsingStrategy(std::string option_name = "",
                                          std::string option_description = "");

        void SetOption(std::string option_name, std::string option_description);

        const std::string& GetOptionName() const noexcept;

        void SetOptionName(std::string option_name);

        const std::string& GetOptionDescription() const noexcept;

        void SetOptionDescription(std::string option_description);

    public:
        std::string m_option_description{""};
        std::string m_option_name{""};
    };
}



