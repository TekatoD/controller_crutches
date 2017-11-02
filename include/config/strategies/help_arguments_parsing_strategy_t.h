/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#pragma once


#include "config/single_option_arguments_parsing_strategy_t.h"

namespace drwn {
    class help_arguments_parsing_strategy_t
            : public single_option_arguments_parsing_strategy_t {
    public:
        help_arguments_parsing_strategy_t();

        void define_description(boost::program_options::options_description& desc) override;

        void apply_variables(const boost::program_options::variables_map& vm) override;

        bool is_help_requested() const;

        void set_help_requested(bool help_requested);

    private:
        bool m_help_requested;

    };
}



