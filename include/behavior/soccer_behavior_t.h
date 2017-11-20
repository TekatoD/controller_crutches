/// \autor arssivka
/// \date 11/13/17

#pragma once


#include <tool/rate_t.h>
#include <localization/field_map_t.h>
#include "behavior_t.h"

namespace drwn {
    class camera_t;
    class vision_t;

    class head_t;
    class walking_t;
    class action_t;
    class kicking_t;

    class localization_t;
    class buttons_t;
    class LEDs_t;
    class game_controller_t;

    class ball_tracker_t;
    class ball_searcher_t;
    class ball_follower_t;
    class go_to_t;

    class soccer_behavior_t : public behavior_t {
        enum class state_t {
            UNKNOWN,
            DISABLED,
            IDLE,
            STANDING_UP,
            GO_TO_STARTING_POSITION,
            KICK_OFF,
            FINDING_BALL
        };

    public:
        explicit soccer_behavior_t();

        void process() override;

        ~soccer_behavior_t() override;

    private:

        void process_decision();

        void process_buttons();

        void process_game_controller();

        void process_cv();

    private:
        state_t m_state{state_t::UNKNOWN};

        bool m_behavior_active{false};
        bool m_manual_penalised{false};
        steady_rate_t m_rate_processing_behavior{std::chrono::milliseconds(10)};
        steady_rate_t m_rate_buttons_check{std::chrono::milliseconds(100)};

        camera_t* m_camera{nullptr};
        vision_t* m_vision{nullptr};
        head_t* m_head{nullptr};
        walking_t* m_walking{nullptr};
        action_t* m_action{nullptr};
        kicking_t* m_kicking{nullptr};
        localization_t* m_localization{nullptr};
        buttons_t* m_buttons{nullptr};
        LEDs_t* m_LEDs{nullptr};
        game_controller_t* m_game_controller{nullptr};
        field_map_t* m_field{nullptr};

        ball_tracker_t* m_tracker{nullptr};
        ball_searcher_t* m_searcher{nullptr};
        ball_follower_t* m_follower{nullptr};
        go_to_t* m_goto{nullptr};


        void check_rate();

        void process_localization();

    };
}



