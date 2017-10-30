/*!
 *  \autor arssivka
 *  \date 10/27/17
 */
#pragma once


#include "config/SingleOptionArgumentsParsingStrategy.h"

namespace Robot {
    class DebugModeArgumentsParsingStrategy
            : public SingleOptionArgumentsParsingStrategy {
    public:
        void DefineDescription(boost::program_options::options_description& desc) override;

        void ApplyVariables(const boost::program_options::variables_map& vm) override;

        bool IsDebugEnabled() const noexcept;

        void EnableDebug(bool debug) noexcept;

        operator bool() const noexcept;

    private:
        bool m_debug{false};
    };
}



