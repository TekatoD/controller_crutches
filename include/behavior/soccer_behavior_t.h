/// \autor arssivka
/// \date 11/13/17

#pragma once


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

    class soccer_behavior_t : public behavior_t {
    public:
        explicit soccer_behavior_t();

        void process() override;

        ~soccer_behavior_t() override;

    private:
        void process_cv();

    private:
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


    };
}



