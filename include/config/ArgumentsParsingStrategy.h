/*!
 *  \autor arssivka
 *  \date 10/27/17
 */
#pragma once


#include <boost/program_options.hpp>

namespace Robot {
    class ArgumentsParsingStrategy {
    public:
        virtual void DefineDescription(boost::program_options::options_description& desc) = 0;

        virtual void ApplyVariables(const boost::program_options::variables_map& vm) = 0;

        virtual ~ArgumentsParsingStrategy() = default;
    };
}



