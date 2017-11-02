/*!
 *  \autor arssivka
 *  \date 10/27/17
 */
#pragma once


#include <boost/program_options.hpp>

namespace drwn {
    class arguments_parsing_strategy_t {
    public:
        virtual void define_description(boost::program_options::options_description& desc) = 0;

        virtual void apply_variables(const boost::program_options::variables_map& vm) = 0;

        virtual ~arguments_parsing_strategy_t() = default;
    };
}



