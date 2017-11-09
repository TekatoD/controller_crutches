/**
 *  @autor arssivka
 *  @date 11/9/17
 */

#include "config/strategies/white_ball_vision_processor_arhuments_parsing_strategy_t.h"

using namespace drwn;

void white_ball_vision_processor_arhuments_parsing_strategy_t::define_description(
        boost::program_options::options_description& desc) {
    namespace po = boost::program_options;
    
    desc.add_options()
            ("display-cv-pipeline", po::value(&m_display_images_enabled)->zero_tokens(), "display results of cv pipeline")
            ("dump-cv-pipeline", po::value(&m_dump_images_enabled)->zero_tokens(), "save state cv pipeline to series of images")
            ("dump-cv-path", po::value(&m_dump_images_path), "cv pipeline dump directory");
}

void white_ball_vision_processor_arhuments_parsing_strategy_t::apply_variables(
        const boost::program_options::variables_map& vm) {
    // Empty method
}

bool white_ball_vision_processor_arhuments_parsing_strategy_t::is_dump_images_enabled() const {
    return m_dump_images_enabled;
}

void white_ball_vision_processor_arhuments_parsing_strategy_t::enable_dump_images(bool dump_images_enabled) {
    m_dump_images_enabled = dump_images_enabled;
}

bool white_ball_vision_processor_arhuments_parsing_strategy_t::is_display_images_enabled() const {
    return m_display_images_enabled;
}

void white_ball_vision_processor_arhuments_parsing_strategy_t::enable_display_images(bool display_images_enabled) {
    m_display_images_enabled = display_images_enabled;
}

const std::string& white_ball_vision_processor_arhuments_parsing_strategy_t::get_dump_images_path() const {
    return m_dump_images_path;
}

void
white_ball_vision_processor_arhuments_parsing_strategy_t::set_dump_images_path(const std::string& dump_images_path) {
    m_dump_images_path = dump_images_path;
}
