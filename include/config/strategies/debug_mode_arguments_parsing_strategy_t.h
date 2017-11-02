/*!
 *  \autor arssivka
 *  \date 10/27/17
 */
#pragma once


#include "config/single_option_arguments_parsing_strategy_t.h"

namespace drwn {
    class debug_mode_arguments_parsing_strategy_t
            : public single_option_arguments_parsing_strategy_t {
    public:
        void define_description(boost::program_options::options_description& desc) override;

        void apply_variables(const boost::program_options::variables_map& vm) override;

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

        operator bool() const noexcept;

    private:
        bool m_debug{false};
    };
}



