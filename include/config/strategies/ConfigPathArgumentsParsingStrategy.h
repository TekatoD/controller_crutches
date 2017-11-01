/*!
 *  \autor arssivka
 *  \date 10/27/17
 */
#pragma once


#include "config/SingleOptionArgumentsParsingStrategy.h"

namespace drwn {
    class ConfigPathArgumentsParsingStrategy : public SingleOptionArgumentsParsingStrategy {
    public:

        void DefineDescription(boost::program_options::options_description& desc) override;

        void ApplyVariables(const boost::program_options::variables_map& vm) override;

        const std::string& GetPath() const;

        void SetPath(const std::string& path);

        operator const std::string&() const noexcept;

    private:
        std::string m_path{""};

    };
}



