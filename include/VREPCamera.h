#ifndef _VREP_CAMERA_H_
#define _VREP_CAMERA_H_

#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING

extern "C" {
    #include "extApi.h"
    #include "extApiPlatform.h"
}

#include "Image.h"
#include "ImgProcess.h"
#include "BaseCamera.h"

namespace Robot {
    class VREPCamera : public BaseCamera {
    public:
        VREPCamera(int width, int height, const char* sensorName, const char* remoteUrl, int portNum);
        ~VREPCamera();
        
        void CaptureFrame();
        
        FrameBuffer* getFrameBuffer();
        Image* getYUVFrame();
        Image* getRGBFrame();
        Image* getHSVFrame();
        Image* getBGRAFrame();
        
        // TODO: Debug
        unsigned char* getImageBuffer() { return (unsigned char*)m_imageBuffer; }
        
        int getWidth() const { return m_res[0]; }
        int getHeight() const { return m_res[1]; }
        
    private:
        FrameBuffer* m_fbuffer;
        simxUChar* m_imageBuffer;
        
        int m_clientId;
        simxInt m_res[2];
        simxInt m_sensorHandle;
    };
}

#endif