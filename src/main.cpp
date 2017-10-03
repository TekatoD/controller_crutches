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

#include <GameController.h>
#include <GoTo.h>
#include <SoccerBehavior.h>
#include <VrepConnector.h>
#include <VrepCM730.h>
#include <GoalieBehavior.h>
#include <LinuxCM730.h>
#include <LinuxCamera.h>
#include <motion/MotionManager.h>
#include <motion/modules/Walking.h>
#include <LinuxMotionTimer.h>
#include <motion/modules/Action.h>
#include <motion/modules/Head.h>
#include <motion/Kinematics.h>
#include <thread>


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

    try {
        VrepCM730* vrepCM730 = static_cast<VrepCM730*>(cm730);
        vrepConnector.Connect();
        vrepCM730->SetClientId(vrepConnector.GetClientID());
        cm730->Connect();
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

    sleep(1);
    StateMachine::GetInstance()->Enable();
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
//
//    Action::GetInstance()->Start(12);
//    Action::GetInstance()->Start(13);

//


    while (!finish) {
        // Update game controller
//        GameController::GetInstance()->Update();

//         Update state machine
//        StateMachine::GetInstance()->Check(cm730);
//        if (!Action::GetInstance()->IsRunning()) {
//            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
//            Walking::GetInstance()->X_MOVE_AMPLITUDE = 20.0;
////            Walking::GetInstance()->A_MOVE_AMPLITUDE = 20.0;
//            Walking::GetInstance()->Start();
//        }


//        Matrix4x4f out;
//        int pan = cm730->m_BulkReadData[JointData::ID_HEAD_PAN].ReadWord(MX28::P_PRESENT_POSITION_L);
//        int tilt = cm730->m_BulkReadData[JointData::ID_HEAD_TILT].ReadWord(MX28::P_PRESENT_POSITION_L);
//        Head::GetInstance()->MoveByAngle(10, 0);
//    cm730->WriteWord(JointData::ID_HEAD_PAN, MX28::P_PRESENT_POSITION_L, MX28::Angle2Value(60), &error);
//        Kinematics::ComputeHeadForwardKinematics(out, (MX28::Value2Angle(pan) * M_PI) / 180, (MX28::Value2Angle(tilt) * M_PI) / 180);
//        std::cout << "pan " << MX28::Value2Angle(pan) << std::endl;
//        std::cout << "tilt " << MX28::Value2Angle(tilt) << std::endl;
//        std::cout << "---------matrix---------" << std::endl;
//        std::cout << out << std::endl;
//        int err;
//        cm730->WriteWord(JointData::ID_L_HIP_PITCH, MX28::P_PRESENT_POSITION_L, MX28::Angle2Value(80), &err);
//        bool xyi = true;
//        Pose2D od = Walking::GetInstance()->GetOdo();
//        if (od.X() != 0 || od.Y() != 0 || od.Theta() != 0 || xyi) {
//            Matrix4x4f out_leg;
//            int pelvis = cm730->m_BulkReadData[JointData::ID_R_HIP_YAW].ReadWord(MX28::P_PRESENT_POSITION_L);
//            int tigh_roll = cm730->m_BulkReadData[JointData::ID_R_HIP_ROLL].ReadWord(MX28::P_PRESENT_POSITION_L);
//            int tigh_pitch = cm730->m_BulkReadData[JointData::ID_R_HIP_PITCH].ReadWord(MX28::P_PRESENT_POSITION_L);
//            int knee_pitch = cm730->m_BulkReadData[JointData::ID_R_KNEE].ReadWord(MX28::P_PRESENT_POSITION_L);
//            int ankle_pitch = cm730->m_BulkReadData[JointData::ID_R_ANKLE_PITCH].ReadWord(MX28::P_PRESENT_POSITION_L);
//            int ankle_roll = cm730->m_BulkReadData[JointData::ID_R_ANKLE_ROLL].ReadWord(MX28::P_PRESENT_POSITION_L);
//            Kinematics::ComputeLegForwardKinematics(out_leg, (MX28::Value2Angle(pelvis) * M_PI) / 180,
//                                                    (MX28::Value2Angle(tigh_roll) * M_PI) / 180,
//                                                    (MX28::Value2Angle(tigh_pitch) * M_PI) / 180,
//                                                    (MX28::Value2Angle(knee_pitch) * M_PI) / 180,
//                                                    (MX28::Value2Angle(ankle_pitch) * M_PI) / 180,
//                                                    (MX28::Value2Angle(ankle_roll) * M_PI) / 180);
//
//            std::cout << "odo X: " << od.X() <<  "odo Y: " << od.Y() <<  "odo Theta: " << od.Theta() << std::endl;

//            std::cout << "pelvis " << MX28::Value2Angle(pelvis) << std::endl;
//            std::cout << "tigh_roll " << MX28::Value2Angle(tigh_roll) << std::endl;
//            std::cout << "tigh_pitch " << MX28::Value2Angle(tigh_pitch) << std::endl;
//            std::cout << "knee_pitch " << MX28::Value2Angle(knee_pitch) << std::endl;
//            std::cout << "ankle_pitch " << MX28::Value2Angle(ankle_pitch) << std::endl;
//            std::cout << "ankle_roll " << MX28::Value2Angle(ankle_roll) << std::endl;
//            std::cout << "---------matrix---------" << std::endl;
//            std::cout << out_leg << std::endl;
//        }
//        Walking::GetInstance()->ResetOdo(Pose2D(0, 0, 0));


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
    }
    vrepConnector.Disconnect();

    return 0;
}
