#include "VREPCamera.h"
#include <iostream>

extern "C" {
    #include "extApi.c"
    #include "extApiPlatform.c"
}

using namespace Robot;

VREPCamera::VREPCamera(int width, int height, const char* sensorName, const char* remoteUrl, int portNum)
    : m_fbuffer(nullptr), m_imageBuffer(nullptr)
{
    // Connect to V-REP remote api
    m_clientId = simxStart((const simxChar*)remoteUrl, portNum, true, true, 2000, 5);
    
    if (m_clientId == -1) {
        // TODO: throw something meaningful
        throw;
    }
    
    std::cout << "Connected to V-Rep" << std::endl;
    
    // Get vision sensor handle
    int status = simxGetObjectHandle(
        m_clientId, sensorName, &m_sensorHandle, simx_opmode_blocking
    );
    
    if (status != simx_return_ok) {
        // TODO: throw something
        throw;
    }
    
    std::cout << "Got sensor handle" << std::endl;
    
    // setup streaming
    int visStream = simxGetVisionSensorImage(
        m_clientId, m_sensorHandle, m_res, &m_imageBuffer, 0, simx_opmode_streaming
    );
    
    std::cout << "Set up streaming: " << visStream << std::endl;
    
    m_res[0] = width;
    m_res[1] = height;
    m_fbuffer = new FrameBuffer(width, height);
}

VREPCamera::~VREPCamera()
{
    //TODO: Use framebuffer instead
    simxReleaseBuffer(m_imageBuffer);
    simxFinish(m_clientId);
    
    if (m_fbuffer != nullptr) {
        delete m_fbuffer;
    }
}

void VREPCamera::CaptureFrame()
{
    // TODO: Convert from RGB to BGR
    // TODO: Flip image
    int visStream = simxGetVisionSensorImage(
        m_clientId, m_sensorHandle, m_res, &m_imageBuffer, 0, simx_opmode_buffer
    );
    
    if (visStream == simx_return_ok) {
        // Put image to framebuffer;
        m_fbuffer->m_RGBFrame->m_ImageData = (unsigned char*)m_imageBuffer;
    }
    
    // TODO: Debug?, sleep for some time
    extApi_sleepMs(10);
}

FrameBuffer* VREPCamera::getFrameBuffer()
{
    return m_fbuffer;
}

Image* VREPCamera::getYUVFrame()
{
    return m_fbuffer->m_YUVFrame;
}

Image* VREPCamera::getRGBFrame()
{
    return m_fbuffer->m_RGBFrame;
}

Image* VREPCamera::getHSVFrame()
{
    return m_fbuffer->m_HSVFrame;
}

Image* VREPCamera::getBGRAFrame()
{
    return m_fbuffer->m_BGRAFrame;
}
