/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <libgen.h>
#include <signal.h>
#include <memory>

#include "LinuxDARwIn.h"

#include "StatusCheck.h"
#include <opencv2/opencv.hpp>
#include <Vision.h>
#include <VisionUtils.h>

#define MOTION_FILE_PATH    "res/motion_4096.bin"
#define INI_FILE_PATH       "res/config.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

using namespace Robot;
using namespace ant;

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);


void change_current_dir() {
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
        if (chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}


bool finish = false;


void sighandler(int sig) {
    finish = true;
}


int main(void) {
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    std::unique_ptr<minIni> ini(new minIni(INI_FILE_PATH));

    std::unique_ptr<Image> rgb_output(new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE));

//    I think it won't need in future
//    LinuxCamera::GetInstance()->Initialize(0);
//    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
//    LinuxCamera::GetInstance()->LoadINISettings(ini.get());                   // load from ini

    std::unique_ptr<ColorFinder> ball_finder(new ColorFinder());
    ball_finder->LoadINISettings(ini.get());

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();

    //////////////////// Framework Initialize ////////////////////////////
    if (MotionManager::GetInstance()->Initialize(&cm730) == false) {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if (MotionManager::GetInstance()->Initialize(&cm730) == false) {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini.get());

    MotionManager::GetInstance()->AddModule((MotionModule*) Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Walking::GetInstance());

    LinuxMotionTimer* motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////

    MotionManager::GetInstance()->LoadINISettings(ini.get());

    int firm_ver = 0;
    if (cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0) != CM730::SUCCESS) {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }

    if (0 < firm_ver && firm_ver < 27) {
        fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
    } else if (27 <= firm_ver) {
        Action::GetInstance()->LoadFile((char*) MOTION_FILE_PATH);
    } else {
        exit(0);
    }
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);

    Action::GetInstance()->Start(15);
    while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);

    Vision vision("../res/vision.json"); // config vision
    cv::VideoCapture cap(0);
    cv::Mat frame;
    while (!finish) {
        StatusCheck::Check(cm730);

        if (StatusCheck::m_is_started == 0)
            continue;

        cap >> frame;

        ant::vision_utils::rot90(frame,0); // free rotation 0, 90, 180 degrees in case of revert camera

        if(frame.empty()) continue;
        vision.setFrame(std::move(frame));

        cv::Mat field = vision.fieldDetect(); // 1 color mask of field
        std::vector<cv::Vec4i> lines = vision.lineDetect(); // lines line(0) line(1) - x0,y0;line(2) line(3) - x1,y1
        cv::Rect ball = vision.ballDetect(); // rect with ball inside
        std::vector<cv::Vec3d> angles = vision.angleDetect(); // angle(0) - x, angle(1) - y, angle(2) - angle value

        tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

        if (Action::GetInstance()->IsRunning() == 0) {
            Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

            follower.Process(tracker.ball_position);
            if (follower.KickBall != 0) {
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                if (follower.KickBall == -1) {
                    Action::GetInstance()->Start(12);   // RIGHT KICK
                    fprintf(stderr, "RightKick! \n");
                } else if (follower.KickBall == 1) {
                    Action::GetInstance()->Start(13);   // LEFT KICK
                    fprintf(stderr, "LeftKick! \n");
                }
            }
        }
//        std::cerr << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_X_L) << " "
//                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_Y_L) << " "
//                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_Z_L) << " "
//                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_ACCEL_X_L) << " "
//                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_ACCEL_Y_L) << " "
//                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_ACCEL_Z_L) << std::endl;
    }

    return 0;
}
