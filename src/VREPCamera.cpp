#include "VREPCamera.h"
#include <iostream>
#include <stdexcept>

extern "C" {
    #include "extApi.c"
    #include "extApiPlatform.c"
}

using namespace Robot;

VREPCamera::VREPCamera(int width, int height, const char* sensorName, const char* remoteUrl, int portNum)
{
    // Connect to V-REP remote api
    m_clientId = simxStart((const simxChar*)remoteUrl, portNum, true, true, 2000, 5);
    
    if (m_clientId == -1) {
        throw std::runtime_error("VREPCamera failed to connect to V-Rep");
    }
    
    m_cold = false;
    std::cout << "Connected to V-Rep" << std::endl;
    
    cameraStreamInit(width, height, sensorName);
}

VREPCamera::VREPCamera(int width, int height, const char* sensorName, int clientId)
{ 
    if (clientId == -1) {
        throw std::runtime_error("VREPCamera invalid clientId");
    }
    m_cold = true;
    m_clientId = clientId;
    
    cameraStreamInit(width, height, sensorName);
}

VREPCamera::~VREPCamera()
{
    simxReleaseBuffer(m_imageBuffer);
    if (!m_cold) {
        simxFinish(m_clientId);
    }
    
    delete m_fbuffer;
}

void VREPCamera::cameraStreamInit(int w, int h, const char* sensorName)
{
    // Get vision sensor handle
    int status = simxGetObjectHandle(
        m_clientId, sensorName, &m_sensorHandle, simx_opmode_blocking
    );
    
    if (status != simx_return_ok) {
        throw std::runtime_error("VREPCamera can't get sensor handle. Check sensorName");
    }
    
    std::cout << "Got sensor handle" << std::endl;
    
    // setup streaming
    int visStream = simxGetVisionSensorImage(
        m_clientId, m_sensorHandle, m_res, &m_imageBuffer, 0, simx_opmode_streaming
    );
    
    std::cout << "Set up streaming: " << visStream << std::endl;
    
    m_res[0] = w;
    m_res[1] = h;
    m_fbuffer = new FrameBuffer(m_res[0], m_res[1]);
}

void VREPCamera::CaptureFrame()
{
    int visStream = simxGetVisionSensorImage(
        m_clientId, m_sensorHandle, m_res, &m_imageBuffer, 0, simx_opmode_buffer
    );
    
    if (visStream == simx_return_ok) {
        // Put image to framebuffer, flip it vertically and convert to BGR
        m_fbuffer->m_RGBFrame->m_ImageData = (unsigned char*)m_imageBuffer;
        ImgProcess::VFlipRGB(m_fbuffer->m_RGBFrame);
        ImgProcess::RGBtoBGR(m_fbuffer);
    }
    
    // TODO: Debug?, sleep for some time
    if (!m_cold) {
        extApi_sleepMs(10);
    }
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

Image* VREPCamera::getBGRFrame()
{
    return m_fbuffer->m_BGRFrame;
}

Image* VREPCamera::getBGRAFrame()
{
    return m_fbuffer->m_BGRAFrame;
}
