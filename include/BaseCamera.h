#ifndef _BASE_CAMERA_H_
#define _BASE_CAMERA_H_

#include "Image.h"

namespace Robot {

    class BaseCamera {
    public:
        virtual FrameBuffer* getFrameBuffer();
        virtual Image* getYUVFrame();
        virtual Image* getRGBFrame();
        virtual Image* getHSVFrame();
        virtual Image* getBGRAFrame();
    };
}


#endif