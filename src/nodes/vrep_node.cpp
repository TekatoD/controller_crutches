#include "VREPCamera.h"
#include "Vision.h"
#include "VisionUtils.h"

#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
#include "extApi.h"
#include "extApi.c"
#include "extApiPlatform.h"
#include "extApiPlatform.c"
}


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    int portNb = 19997;
    int clientId = simxStart((const simxChar*)"127.0.0.1", portNb, true, true, 2000, 5);
    const char* visionSensorName = "camera";
    
    Robot::VREPCamera camera(320, 240, visionSensorName);
    camera.connect(clientId);
    int w = camera.getWidth();
    int h = camera.getHeight();
    
    ant::Vision vision("../res/vision_cfg/");
    cv::namedWindow("camera_image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("line_image", cv::WINDOW_AUTOSIZE);
    while (true) {
        camera.CaptureFrame();
        
        unsigned char* imgBuff = camera.getBGRFrame()->m_ImageData;
        
        if (imgBuff) {
            cv::Mat frame(cv::Size(w, h), CV_8UC3, imgBuff, cv::Mat::AUTO_STEP);
            
            cv::imshow("camera_image", frame);
            
            //ant::vision_utils::rot90(frame, 0);
            vision.setFrame(frame);
            std::vector<cv::Vec4i> lines = vision.lineDetect();
            
            std::cout << "Line count: " << lines.size() << std::endl;

            for (auto& line : lines) {
                cv::Point p1(line[0], line[1]);
                cv::Point p2(line[2], line[3]);

                cv::line(frame, p1, p2, cv::Scalar(0, 0, 255), 5);
            }
            
            cv::imshow("line_image", frame);
            cv::waitKey(1);
        }
        
        extApi_sleepMs(10);
    }
    
    simxFinish(clientId);
    
    /*
    if (clientID != -1) {
        std::cout << "Connection to V-Rep successful!" << std::endl;
        simxInt visionSensor = 0;
        
        int status = simxGetObjectHandle(
            clientID, visionSensorName, &visionSensor, simx_opmode_blocking
        );
        
        if (status == simx_return_ok) {
            std::cout << "Acquired vision sensor handle" << std::endl;
            
            simxUChar* image;
            simxInt res[2];
            int visStream = simxGetVisionSensorImage(
                clientID, visionSensor, res, &image, 0, simx_opmode_streaming
            );
            std::cout << "Vision stream returned: " << visStream << std::endl;
            
            cv::namedWindow("sensor_image", cv::WINDOW_AUTOSIZE);
            cv::Mat cv_image = cv::Mat::eye(256, 256, CV_8U);
            while (simxGetConnectionId(clientID) != -1) {
                visStream = simxGetVisionSensorImage(
                    clientID, visionSensor, res, &image, 0, simx_opmode_buffer
                );
                
                if (visStream == simx_return_ok) {
                    cv_image = vrep_to_opencv(image, res);
                }
                
                cv::imshow("sensor_image", cv_image);
                cv::waitKey(1);
                extApi_sleepMs(100);
            }
            
            simxReleaseBuffer(image);
        } else {
            std::cout << "Failed to acquire vision sensor handle" << std::endl;
        }
        
        simxFinish(clientID);
    }
    */
    
    return 0;
}
