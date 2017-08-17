#ifndef _BASE_CAMERA_H_
#define _BASE_CAMERA_H_

#include "Image.h"

namespace Robot {

    class BaseCamera {
    public:
        
        virtual void CaptureFrame() = 0;
        
        virtual FrameBuffer* getFrameBuffer() = 0;
        virtual Image* getYUVFrame() = 0;
        virtual Image* getRGBFrame() = 0;
        virtual Image* getHSVFrame() = 0;
        virtual Image* getBGRAFrame() = 0;
    };
}


#endif