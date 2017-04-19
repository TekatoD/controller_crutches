/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include <memory>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "../include/StatusCheck.h"
#include "../include/VisionMode.h"

#define MOTION_FILE_PATH    "res/motion_4096.bin"
#define INI_FILE_PATH       "res/config.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);


void change_current_dir() {
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
        if (chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}


void sighandler(int sig) {
    exit(0);
}


int main(void) {
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    std::unique_ptr<minIni> ini(new minIni(INI_FILE_PATH));
    std::unique_ptr<Image> rgb_output(new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE));

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini.get());                   // load from ini

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini.get());
    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();

    ColorFinder* red_finder = new ColorFinder(0, 15, 45, 0, 0.3, 50.0);
    red_finder->LoadINISettings(ini.get(), "RED");
    httpd::red_finder = red_finder;

    ColorFinder* yellow_finder = new ColorFinder(60, 15, 45, 0, 0.3, 50.0);
    yellow_finder->LoadINISettings(ini.get(), "YELLOW");
    httpd::yellow_finder = yellow_finder;

    ColorFinder* blue_finder = new ColorFinder(225, 15, 45, 0, 0.3, 50.0);
    blue_finder->LoadINISettings(ini.get(), "BLUE");
    httpd::blue_finder = blue_finder;

    httpd::ini = ini.get();

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
    } else
        exit(0);

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);

    Action::GetInstance()->Start(15);
    while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);

    while (true) {
        StatusCheck::Check(cm730);

        Point2D ball_pos, red_pos, yellow_pos, blue_pos;

        if (StatusCheck::m_cur_mode != ROBOPLUS) {
            LinuxCamera::GetInstance()->CaptureFrame();
            memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData,
                   LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        }


        tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

        for (int i = 0; i < rgb_output->m_NumberOfPixels; i++) {
            if (ball_finder->m_result->m_ImageData[i] == 1) {
                rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 0] = 255;
                rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 1] = 128;
                rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 2] = 0;
            }
        }

        if (StatusCheck::m_is_started == 0)
            continue;


        if (Action::GetInstance()->IsRunning() == 0) {
            Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

            follower.Process(tracker.ball_position);

            Point2D piunt(100, 100);
            follower.Process(piunt);
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
        std::cerr << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_X_L) << " "
                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_Y_L) << " "
                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_GYRO_Z_L) << " "
                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_ACCEL_X_L) << " "
                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_ACCEL_Y_L) << " "
                  << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(Robot::CM730::P_ACCEL_Z_L) << std::endl;
    }

    return 0;
}
