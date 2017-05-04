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
#include <chrono>

#include <GameController.h>
#include <Speech.h>
#include <thread>
#include <GoTo.h>
#include "OdometryCollector.h"

#include "LinuxDARwIn.h"

#include "StatusCheck.h"

#define MOTION_FILE_PATH    "res/motion_4096.bin"
#define INI_FILE_PATH       "res/config.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

using namespace Robot;

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

    auto ini = std::make_unique<minIni>(INI_FILE_PATH);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini.get());                   // load from ini

    auto ball_finder = std::make_unique<ColorFinder>();
    ball_finder->LoadINISettings(ini.get());

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
    GoTo goTo = GoTo();

    goTo.LoadINISettings(ini.get());

    //////////////////// Framework Initialize ///////////////////////////
    if (!MotionManager::GetInstance()->Initialize(&cm730)) {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if (!MotionManager::GetInstance()->Initialize(&cm730)) {
            std::cerr << "Fail to initialize Motion Manager!" << std::endl;
            return 1;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini.get());

    MotionManager::GetInstance()->AddModule((MotionModule*) Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Walking::GetInstance());

    auto motion_timer = std::make_unique<LinuxMotionTimer>(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////

    MotionManager::GetInstance()->LoadINISettings(ini.get());

    int firm_ver = 0;
    if (cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0) != CM730::SUCCESS) {
        std::cerr << "Can't read firmware version from Dynamixel ID " << JointData::ID_HEAD_PAN << "!!\n" << std::endl;
        return 1;
    }

    if (0 < firm_ver && firm_ver < 27) {
        std::cerr << "MX-28's firmware is not support 4096 resolution!! \n"
                  << "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n"
                  << std::endl;
        return 1;
    } else if (27 <= firm_ver) {
        Action::GetInstance()->LoadFile((char*) MOTION_FILE_PATH);
    } else {
        return 1;
    }


    ///////////////////// Init game controller //////////////////////////

    GameController::GetInstance()->LoadINISettings(ini.get());
    if (!GameController::GetInstance()->Connect()) {
        std::cerr << "ERROR: Can't connect to game controller!" << std::endl;
        return 1;
    }

    /////////////////////////////////////////////////////////////////////



    ///////////////////////// Init speech ///////////////////////////////
//    if (!Speech::GetInstance()->Start()) {
//        std::cerr << "ERROR: Can't start speech module!" << std::endl;
//        return 1;
//    }
    /////////////////////////////////////////////////////////////////////



    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);

    Action::GetInstance()->Start(15);
    while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);

    while (!finish) {
        // Update game controller
        GameController::GetInstance()->Update();

        // Update state machine
        StatusCheck::Check(cm730);

        if (StatusCheck::m_is_started == 0) {
            continue;
        }

        // Update CV
        LinuxCamera::GetInstance()->CaptureFrame();
        tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

        if (Action::GetInstance()->IsRunning() == 0) {
            // Switch to head and walking after action
            Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
            // Follow the ball
            Pose2D odo = Walking::GetInstance()->GetOdo();
            Pose2D target(0, 400, 0.0);
            goTo.Process(target - odo);

            std::cout << "odo: " << odo << "; t: " << target << "; d: " << target - odo << std::endl;

//            follower.Process(tracker.ball_position);
            ///////////////
//            std::ofstream out;
//            out.open("odo.txt", std::ios::app);
//            out << Walking::GetInstance()->Get() << std::endl;
//            out.close();
            //////////////
            if (follower.KickBall != 0) {
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                // Kick the ball
                if (follower.KickBall == -1) {
                    Action::GetInstance()->Start(12);   // RIGHT KICK
                } else if (follower.KickBall == 1) {
                    Action::GetInstance()->Start(13);   // LEFT KICK
                }
            }
        }
    }


    return 0;
}
