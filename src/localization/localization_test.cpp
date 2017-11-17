/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */
/*
 *  For reference only
 *
#include <stdio.h>
#include <unistd.h>
#include <libgen.h>
#include <signal.h>
#include <memory>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

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
#include <Vision.h>
#include <VisionUtils.h>
#include <motion/Kinematics.h>
#include <localization/particle_filter_t.h>
#include <OdometryCollector.h>

#include "StateMachine.h"

#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
#include "extApi.h"
#include "extApiPlatform.h"
}

#define MOTION_FILE_PATH    "res/motion_4096.bin"
#define INI_FILE_PATH       "res/config.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

using namespace Robot;
using namespace localization;

using Clock =  std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<float>;

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

    VREPCamera camera(320, 240, "camera");

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
////    Action::GetInstance()->Start(12);
//    Action::GetInstance()->Start(13);

    auto DarwinHead = Head::GetInstance();

    ant::Vision vision("./res/vision_cfg/");
    cv::namedWindow("line_image", cv::WINDOW_AUTOSIZE);

    localization::particle_filter_t particleFilter(&ini);
    Pose2D initPose = particleFilter.get_top_particle().pose;
    Walking::GetInstance()->SetOdo(initPose);

    Pose2D oldRobotPose(initPose), robotPose(initPose);

    // Movement noise (odometry movement model format)
    // Rotation1, translation, Rotation2
    Eigen::Vector3f movementNoise = {0.01, 50.0f, 0.01};
    // Measurement noise (range-bearing measurement model format)
    // range (in mm), bearing (in radians)
    Eigen::Vector3f measurementNoise = {600.0f, 0.05, 0.0f};

    while (!finish) {
        // Update game controller
//        GameController::GetInstance()->Update();

        // Update state machine
        StateMachine::GetInstance()->Check(cm730);
//
        if (!Action::GetInstance()->IsRunning()) {
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
            Walking::GetInstance()->X_MOVE_AMPLITUDE = 20.0;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = 20.0;
            Walking::GetInstance()->Start();
        }

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
//            case ROLES_COUNT:break;.0f
//       float rx, ry, rtheta;



        std::cout << "=== Begin particle filter cycle ===" << std::endl;
        float rx, ry, rtheta;
        robotPose = Walking::GetInstance()->GetOdo();

        std::cout << "Old: " << oldRobotPose << ", Received: " << robotPose << std::endl;
        rx = robotPose.X();
        ry = robotPose.Y();
        rtheta = robotPose.Theta();

        auto pf_pred_start = Clock::now();
        Eigen::Vector3f odometryCommand = particleFilter.get_odometry_command(oldRobotPose, robotPose);
        particleFilter.predict(odometryCommand, movementNoise);
        auto pf_predict_diff = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - pf_pred_start);

        //std::cout << "Successful prediction step using odometry command: " << std::endl;
        //std::cout << odometryCommand << std::endl;


        float HeightFromGround = Robot::Kinematics::LEG_LENGTH + 122.2f + 50.5f + Robot::Kinematics::CAMERA_OFFSET_Z;
        //HeightFromGround /= 1000.0f;
        // Z
        float HeadPan = (DarwinHead->GetPanAngle()) * (M_PI / 180.0f);
        // Y
        float HeadTilt = (DarwinHead->GetTiltAngle()) * (M_PI / 180.0f);

        Matrix4x4f HeadTransformE;
        cv::Mat HeadTransform;
        Robot::Kinematics::ComputeHeadForwardKinematics(HeadTransformE, HeadPan, HeadTilt);
        cv::eigen2cv(HeadTransformE, HeadTransform);

        //std::cout << "HeadTransform: " << HeadTransform << std::endl;

        cv::Mat R = HeadTransform(cv::Range(0, 3), cv::Range(0, 3));

        // Translation is in mm
        cv::Mat t = HeadTransform(cv::Range(0, 3), cv::Range(3, 4));
        //t /= 1000.0f;
        t.at<float>(2, 0) += HeightFromGround;

        // cv::Mat t = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, HeightFromGround);

        ant::vision_utils::camera_parameters_t params(R, t, 0.02, 1.0f, 0.0f, 320.0f/2.0f, 240.0f/2.0f);
        ant::vision_utils::camera_projection_t cameraToGround(params);

        camera.CaptureFrame();
        unsigned char* imgBuff = camera.getBGRFrame()->m_ImageData;
        if (imgBuff) {
            // Mats that take buffer in the constructor don't release it
            cv::Mat frame(camera.getHeight(), camera.getWidth(), CV_8UC3, imgBuff);

            vision.setFrame(frame);
            std::vector<cv::Vec4i> lines = vision.lineDetect();

            particle_filter_t::measurement_bundle rangeBearingData;
            for (auto& line : lines) {
                if (line[0] > camera.getWidth() || line[2] > camera.getWidth() || line[1] > camera.getHeight() || line[3] > camera.getHeight()) {
                    // why
                    continue;
                }
                // Point for viz
                cv::Point p1(line[0], line[1]);
                cv::Point p2(line[2], line[3]);

                cv::line(frame, p1, p2, cv::Scalar(0, 0, 255), 5);

                cv::Mat mp1, mp2, gp1, gp2;
                mp1 = (cv::Mat_<float>(3, 1) << line[0], line[1], 1);
                mp2 = (cv::Mat_<float>(3, 1) << line[2], line[3], 1);

                gp1 = cameraToGround.image_to_world_explicit(mp1, R, t, 320.0f, 240.0f, 46.0f);
                gp2 = cameraToGround.image_to_world_explicit(mp2, R, t, 320.0f, 240.0f, 46.0f);
                float x1 = gp1.at<float>(0, 0);
                // change direction of y axis
                float y1 = -1*gp1.at<float>(1, 0);

                float x2 = gp2.at<float>(0, 0);
                float y2 = -1*gp2.at<float>(1, 0);

                // Ignore lines that are projected  too far (Like goal keeper gate lines)
                if ((x1 < 0.0f) || (x2 < 0.0f)) {
                    continue;
                }


                Eigen::Vector4f lineRangeBearing = particle_filter_t::get_line_range_bearing(robotPose, x1, y1, x2, y2);

                rangeBearingData.push_back(lineRangeBearing);
            }

            auto pf_correct_start = Clock::now();
            particleFilter.correct(rangeBearingData, measurementNoise);
            auto pf_correct_diff = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - pf_correct_start);

            auto pf_resample_start = Clock::now();
            particleFilter.resample();
            auto pf_resample_diff = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - pf_resample_start);

            std::cout << "==== PF pose ==" << std::endl;
            std::cout << "Pose mean: " << particleFilter.get_pose_mean() << std::endl;
            std::cout << "Pose std dev: " << particleFilter.get_pose_std_dev() << std::endl;
            std::cout << "Highest weight particle pose: " << particleFilter.get_top_particle().pose << std::endl;


            std::cout << "PF prediction: " << pf_predict_diff.count() << "ms" << std::endl;
            std::cout << "PF correction: " << pf_correct_diff.count() << "ms" << std::endl;
            std::cout << "PF resampling: " << pf_resample_diff.count() << "ms" << std::endl;
            std::cout << "PF total time: " << (pf_predict_diff + pf_correct_diff + pf_resample_diff).count() << "ms" << std::endl;
            std::cout << "=== End particle filter cycle ===" << std::endl;

            cv::imshow("line_image", frame);
            cv::waitKey(1);
        }

        oldRobotPose = robotPose;


    }
    vrepConnector.Disconnect();

    return 0;
}

*/
