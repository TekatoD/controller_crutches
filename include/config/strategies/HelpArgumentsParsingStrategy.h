/*!
 *  \autor arssivka
 *  \date 10/27/17
 */

#pragma once


#include "config/SingleOptionArgumentsParsingStrategy.h"

namespace drwn {
    class HelpArgumentsParsingStrategy
            : public SingleOptionArgumentsParsingStrategy {
    public:
        HelpArgumentsParsingStrategy();

        void DefineDescription(boost::program_options::options_description& desc) override;

        void ApplyVariables(const boost::program_options::variables_map& vm) override;

        bool IsHelpRequested() const;

        void SetHelpRequested(bool help_requested);

    private:
        bool m_help_requested;

    };
}



