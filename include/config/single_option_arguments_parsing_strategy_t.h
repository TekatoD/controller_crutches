/*!
 *  \autor arssivka
 *  \date 10/27/17
 */
#pragma once


#include <boost/program_options.hpp>
#include "config/arguments_parsing_strategy_t.h"

namespace drwn {
    class single_option_arguments_parsing_strategy_t
            : public arguments_parsing_strategy_t {
    public:
        explicit single_option_arguments_parsing_strategy_t(std::string option_name = "",
                                          std::string option_description = "");

        void set_option(std::string option_name, std::string option_description);

        const std::string& get_option_name() const noexcept;

        void set_option_name(std::string option_name);

        const std::string& get_option_description() const noexcept;

        void set_option_description(std::string option_description);

    public:
        std::string m_option_description{""};
        std::string m_option_name{""};
    };
}



