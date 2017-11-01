/*
 *   Camera.h
 *   This class contains the constants about the camera.
 *   Author: ROBOTIS
 *
 */

#ifndef _CAMERA_H_
#define _CAMERA_H_


namespace drwn {
    class Camera {
    public:
        static constexpr float VIEW_V_ANGLE = 46.0; //degree
        static constexpr float VIEW_H_ANGLE = 58.0; //degree

        static constexpr int WIDTH = 320;
        static constexpr int HEIGHT = 240;
    };

}

#endif
