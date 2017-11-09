/**
 *  @autor arssivka
 *  @date 11/9/17
 */

#pragma once

#include <config/arguments_parsing_strategy_t.h>
#include <vision/white_ball_vision_processor_t.h>


namespace drwn {
    class white_ball_vision_processor_arhuments_parsing_strategy_t
            : public arguments_parsing_strategy_t {
    public:
        virtual void define_description(boost::program_options::options_description& desc) override;

        virtual void apply_variables(const boost::program_options::variables_map& vm) override;

    private:
        bool m_dump_images_enabled{false};
        bool m_display_images_enabled{false};
        std::string m_dump_images_path;
    };
}


