/**
 *  @autor tekatod
 *  @date 11/7/17
 */
#pragma once

#include <vision/white_ball_vision_processor_t.h>
#include "config/configuration_strategy_t.h"


namespace drwn {
    class white_ball_vision_processor_configuration_strategy_t
            : public configuration_strategy_t {
    public:

        static constexpr char DEFAULT_SECTION[] = "Vision";

        explicit white_ball_vision_processor_configuration_strategy_t(white_ball_vision_processor_t* ball_vision_processor =
                                                                                nullptr, std::string section = DEFAULT_SECTION);

        void read_config(const boost::property_tree::ptree& prop) override;

        void write_config(boost::property_tree::ptree& prop) const override;



    private:
        white_ball_vision_processor_t* m_white_ball_vision_processor{nullptr};
    public:
        white_ball_vision_processor_t* get_white_ball_vision_processor() const;

        void set_white_ball_vision_processor(white_ball_vision_processor_t* white_ball_vision_processor);
    };
}
