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
#include <opencv2/opencv.hpp>

#include <GameController.h>
#include <GoTo.h>
#include <SoccerBehavior.h>
#include <VrepConnector.h>
#include <VrepCM730.h>
#include <VREPCamera.h>
#include <GoalieBehavior.h>
#include <LinuxCM730.h>
#include <LinuxCamera.h>
#include <motion/MotionManager.h>
#include <motion/modules/Walking.h>
#include <LinuxMotionTimer.h>
#include <motion/modules/Action.h>
#include <motion/modules/Head.h>
#include <Vision.h>
#include <VisionUtils.h>

#include "StateMachine.h"

#define MOTION_FILE_PATH    "res/motion_4096.bin"
#define INI_FILE_PATH       "res/config.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

using namespace Robot;

//LinuxCM730 linux_cm730(U2D_DEV_NAME0);
VrepConnector vrepConnector;
CM730* cm730 = new VrepCM730();


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


int main(int argc, char** argv) {
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();
    
    int width = 320, height = 240;
    VREPCamera camera(width, height, "camera");
    
    try {
        VrepCM730* vrepCM730 = static_cast<VrepCM730*>(cm730);
        vrepConnector.Connect();
        vrepCM730->SetClientId(vrepConnector.GetClientID());
        cm730->Connect();
        
        camera.connect(vrepConnector.GetClientID());
    }
    catch(std::runtime_error& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }


    minIni ini(argc == 1 ? INI_FILE_PATH : argv[1]);

//    LinuxCamera::GetInstance()->Initialize(0);
//    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
//    LinuxCamera::GetInstance()->LoadINISettings(&ini);                   // load from ini

//    auto ball_finder = std::make_unique<ColorFinder>();
//    ball_finder->LoadINISettings(ini.get());

//    BallTracker tracker = BallTracker();
//    BallFollower follower = BallFollower();
//    GoTo goTo = GoTo();

//    goTo.LoadINISettings(ini.get());

    //////////////////// Framework Initialize ///////////////////////////
    /*
    if (!MotionManager::GetInstance()->Initialize(cm730)) {
//        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if (!MotionManager::GetInstance()->Initialize(cm730)) {
            std::cerr << "Fail to initialize Motion Manager!" << std::endl;
            return 1;
        }
    }

    Walking::GetInstance()->LoadINISettings(&ini);

    MotionManager::GetInstance()->AddModule((MotionModule*) Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Walking::GetInstance());

    LinuxMotionTimer motion_timer(MotionManager::GetInstance());
    motion_timer.Start();
    
    /////////////////////////////////////////////////////////////////////

    MotionManager::GetInstance()->LoadINISettings(&ini);
    StateMachine::GetInstance()->LoadINISettings(&ini);

    int firm_ver = 0;
    if (cm730->ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0) != CM730::SUCCESS) {
        std::cerr << "Can't read firmware version from Dynamixel ID " << JointData::ID_HEAD_PAN << "!!\n" << std::endl;
        return 1;
    }

    if (0 < firm_ver && firm_ver < 27) {
        std::cerr << "MX-28's firmware is not support 4096 resolution!! \n"
                  << "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n"
                  << std::endl;
        return 1;
    } else if (27 <= firm_ver) {
        Action::GetInstance()->LoadFile(argc <= 2 ? (char*) MOTION_FILE_PATH : argv[2]);
    } else {
        return 1;
    }
    */

    ///////////////////// Init game controller //////////////////////////

//    GameController::GetInstance()->LoadINISettings(&ini);
//    if (!GameController::GetInstance()->Connect()) {
//        std::cerr << "ERROR: Can't connect to game controller!" << std::endl;
//        return 1;
//    }

    /////////////////////////////////////////////////////////////////////



    ///////////////////////// Init speech ///////////////////////////////
//    if (!Speech::GetInstance()->Start()) {
//        std::cerr << "ERROR: Can't start speech module!" << std::endl;
//        return 1;
//    }
    /////////////////////////////////////////////////////////////////////


//    SoccerBehavior soccer(cm730);
//    MotionManager::GetInstance()->SetEnable(true);
//    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
//    Walking::GetInstance()->X_MOVE_AMPLITUDE = 10.0;
//    Walking::GetInstance()->Start();
//    while (true) {
//        std::cout << "hui" << std::endl;
//    }
//    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
//    MotionManager::GetInstance()->SetEnable(true);
//
//    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);
//
//    SoccerBehavior soccer(cm730);
//    GoalieBehavior goalie;
//
//    soccer.LoadINISettings(&ini);
//    goalie.LoadINISettings(&ini);
//
//    Action::GetInstance()->Start(15);
//    while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);
/*
    sleep(1);
    StateMachine::GetInstance()->Enable();
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
*/
//
////    Action::GetInstance()->Start(12);
//    Action::GetInstance()->Start(13);

    ant::Vision vision("res/vision_cfg");
    cv::namedWindow("camera_image", cv::WINDOW_AUTOSIZE);
    while (!finish) {
        camera.CaptureFrame();
        
        unsigned char* imgBuff = camera.getBGRFrame()->m_ImageData;
        
        if (imgBuff) {
            cv::Mat frame(cv::Size(width, height), CV_8UC3, imgBuff, cv::Mat::AUTO_STEP);
            //cv::imshow("camera_image", frame);
            
            ant::vision_utils::rot90(frame, 0);
            vision.setFrame(frame);
            /*
            std::vector<cv::Vec4i> lines = vision.lineDetect();
            
            for (auto& line : lines) {
                cv::Point p1(line[0], line[1]);
                cv::Point p2(line[2], line[3]);
                
                cv::line(frame, p1, p2, cv::Scalar(0, 0, 255), 5);
            }
            */
            
            cv::imshow("camera_image", frame);
        }
        
        // Update game controller
//        GameController::GetInstance()->Update();

        // Update state machine
        /*
        StateMachine::GetInstance()->Check(cm730);
//
        if (!Action::GetInstance()->IsRunning()) {
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
            Walking::GetInstance()->X_MOVE_AMPLITUDE = 20.0;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = 20.0;
            Walking::GetInstance()->Start();
        }
        */

//        if (StateMachine::GetInstance()->IsStarted() == 0) {
//            continue;
//        }
//
//        switch (StateMachine::GetInstance()->GetRole()) {
//            case ROLE_IDLE:break;
//            case ROLE_SOCCER:
//                soccer.Process();
//                break;
//            case ROLE_PENALTY:break;
//            case ROLE_GOALKEEPER:
//                goalie.Process();
//                break;
//            case ROLES_COUNT:break;
//        }
        cv::waitKey(1);
    }
    vrepConnector.Disconnect();

    return 0;
}
