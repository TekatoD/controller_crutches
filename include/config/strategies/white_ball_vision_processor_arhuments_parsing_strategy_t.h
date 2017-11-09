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
        void define_description(boost::program_options::options_description& desc) override;

        void apply_variables(const boost::program_options::variables_map& vm) override;

        bool is_dump_images_enabled() const;

        void enable_dump_images(bool dump_images_enabled);

        bool is_display_images_enabled() const;

        void enable_display_images(bool display_images_enabled);

        const std::string& get_dump_images_path() const;

        void set_dump_images_path(const std::string& dump_images_path);

    private:
        bool m_dump_images_enabled{false};
        bool m_display_images_enabled{false};
        std::string m_dump_images_path;
    };
}


