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
