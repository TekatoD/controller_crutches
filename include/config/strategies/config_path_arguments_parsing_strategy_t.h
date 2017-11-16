/*!
 *  \autor arssivka
 *  \date 10/27/17
 */
#pragma once


#include "config/single_option_arguments_parsing_strategy_t.h"

namespace drwn {
    class config_path_arguments_parsing_strategy_t : public single_option_arguments_parsing_strategy_t {
    public:

        void define_description(boost::program_options::options_description& desc) override;

        void apply_variables(const boost::program_options::variables_map& vm) override;

        const std::string& get_path() const;

        void set_path(const std::string& path);

        bool is_set() const noexcept;

        operator bool() const noexcept;

        operator const std::string&() const noexcept;

    private:
        std::string m_path{""};

    };
}



