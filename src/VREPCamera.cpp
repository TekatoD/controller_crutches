#include "VREPCamera.h"
#include <iostream>
#include <stdexcept>
#include <cstring>

#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
    #include "extApi.h"
    #include "extApiPlatform.h"
}

using namespace Robot;

VREPCamera::VREPCamera(int width, int height, const char* sensorName) :
    m_width(width), m_height(height), m_sensorName((char*)sensorName)
{ 
    m_clientId = -1;
    m_imageBuffer = 0;
    m_fbuffer = 0;
}

VREPCamera::~VREPCamera()
{
    // Nothing to release if couldn't connect to the sim
    if (m_imageBuffer) {
        simxReleaseBuffer(m_imageBuffer);
    }
    
    if (m_fbuffer) {
        delete m_fbuffer;
    }
}

void VREPCamera::connect(int clientId)
{

    if (clientId == -1) {
        throw std::runtime_error("VREPCamera invalid clientId");
    }
    m_clientId = clientId;
    
    cameraStreamInit(m_width, m_height, m_sensorName);
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
    
    // m_imageBuffer data is automatically released on every call
    int visStream = simxGetVisionSensorImage(
        m_clientId, m_sensorHandle, m_res, &m_imageBuffer, 0, simx_opmode_buffer
    );
    
    if (visStream == simx_return_ok) {
        // Received a broken frame
        if (m_res[0] != m_width || m_res[1] != m_height) {
            std::cout << "V-rep why? w, h: " << m_res[0] << " " << m_res[1] << std::endl;
            m_res[0] = m_width;
            m_res[1] = m_height;
            return;
        }
        // Put image to framebuffer, flip it vertically and convert to BGR
        memcpy(m_fbuffer->m_RGBFrame->m_ImageData, m_imageBuffer, m_fbuffer->m_RGBFrame->m_ImageSize);
        ImgProcess::VFlipRGB(m_fbuffer->m_RGBFrame);
        ImgProcess::RGBtoBGR(m_fbuffer);
    }
    
    //simxSynchronousTrigger(m_clientId);
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
