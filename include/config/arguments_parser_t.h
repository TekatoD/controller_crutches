/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#pragma once


#include <list>
#include "arguments_parsing_strategy_t.h"
#include "command_arguments_t.h"

namespace drwn {
    class arguments_parser_t {
    public:
        explicit arguments_parser_t(std::string caption = "");

        void add_strategy(arguments_parsing_strategy_t& strategy);

        void remove_strategy(const arguments_parsing_strategy_t& strategy);

        bool parse();

        const std::string& get_caption() const noexcept;

        void set_caption(const std::string& caption);

        void show_description_to_stream(std::ostream& stream) const;

        const command_arguments_t& get_arguments() const noexcept;

        void set_arguments(const command_arguments_t& arguments) noexcept;

    private:
        boost::program_options::options_description generate_option_description() const;

    private:
        using strategies_list_t = std::list<arguments_parsing_strategy_t*>;

        strategies_list_t m_strategies;
        command_arguments_t m_arguments;
        std::string m_caption;

        void apply_variables_map(const boost::program_options::variables_map& vm);
    };
}
