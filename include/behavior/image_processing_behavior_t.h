/// \autor arssivka
/// \date 11/13/17

#pragma once


#include "behavior_t.h"

namespace drwn {
    class camera_t;
    class vision_t;

    class image_processing_behavior_t : public behavior_t {
    public:
        image_processing_behavior_t();

        void process() override;

    private:
        camera_t* m_camera{nullptr};
        vision_t* m_vision{nullptr};
    };
}



