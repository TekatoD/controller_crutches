#ifndef _VREP_CAMERA_H_
#define _VREP_CAMERA_H_

#include "Image.h"
#include "ImgProcess.h"
#include "BaseCamera.h"

namespace Robot {
    class VREPCamera : public BaseCamera {
    public:
        VREPCamera(int width, int height, const char* sensorName);
        ~VREPCamera();
        
        void CaptureFrame();
        
        FrameBuffer* getFrameBuffer();
        Image* getYUVFrame();
        Image* getRGBFrame();
        Image* getHSVFrame();
        Image* getBGRFrame();
        Image* getBGRAFrame();
        
        // TODO: Debug
        unsigned char* getImageBuffer() { return (unsigned char*)m_imageBuffer; }
        void connect(int clientId = -1);
        int getWidth() const { return m_res[0]; }
        int getHeight() const { return m_res[1]; }
        
    private:
        void cameraStreamInit(int w, int h, const char* sensorName);
        
        FrameBuffer* m_fbuffer;
        unsigned char* m_imageBuffer;
        int m_width, m_height;
        
        char* m_remoteUrl;
        char* m_sensorName;
        int m_port;
        int m_clientId;
        
        int m_res[2];
        int m_sensorHandle;
    };
}

#endif