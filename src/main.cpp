/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <cstdio>
#include <unistd.h>
#include <libgen.h>
#include <csignal>
#include <memory>

#include <GameController.h>
#include <GoTo.h>
#include <SoccerBehavior.h>
#include <hw/VrepConnector.h>
#include <hw/VrepCM730.h>
#include <GoalieBehavior.h>
#include <hw/LinuxCM730.h>
#include <motion/MotionManager.h>
#include <motion/modules/Walking.h>
#include <LinuxMotionTimer.h>
#include <motion/modules/Action.h>
#include <motion/modules/Head.h>
#include <motion/modules/Kicking.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/support/date_time.hpp>
#include <log/ColouredSink.h>
#include <log/Logger.h>


#include "StateMachine.h"

#define MOTION_FILE_PATH    "res/motion_4096.bin"
#define INI_FILE_PATH       "res/config.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

using namespace Robot;

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;


void change_current_dir() {
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
        if (chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}


bool finish = false;


void sighandler(int /*sig*/) {
    finish = true;
}


int main(int argc, char** argv) {
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    //////////////////// Logging Initialize ///////////////////////////
    using coloured_sink = sinks::synchronous_sink<ColouredSink>;
    auto sink = boost::make_shared<coloured_sink>();
    sink->set_formatter(expr::stream
                                << expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%H:%M:%S.%f")
                                << " [" << logging::trivial::severity
                                << "] " << expr::smessage);
    logging::core::get()->set_filter(
#ifdef DEBUG
            logging::trivial::severity >= logging::trivial::debug
#else
            logging::trivial::severity >= logging::trivial::info
#endif
    );
    logging::core::get()->add_sink(std::move(sink));

    LOG_INFO << "=== Initialization was started ===";


    change_current_dir();

    //LinuxCM730 linux_cm730(U2D_DEV_NAME0);
    VrepConnector vrepConnector;
    VrepCM730 cm730;

    try {
        vrepConnector.Connect();
        cm730.SetClientId(vrepConnector.GetClientID());
        cm730.Connect();
    } catch(std::runtime_error& e) {
        LOG_FATAL << "Error: " << e.what();
        return 1;
    }
    LOG_INFO << "Hardware is ready";

    minIni ini(argc == 1 ? INI_FILE_PATH : argv[1]);

    //////////////////// Framework Initialize ///////////////////////////
    if (!MotionManager::GetInstance()->Initialize(&cm730)) {
//        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if (!MotionManager::GetInstance()->Initialize(&cm730)) {
            LOG_FATAL << "Fail to initialize Motion Manager!";
            return 1;
        }
    }
    LOG_INFO << "Motion manager is ready";

    Walking::GetInstance()->LoadINISettings(&ini);

    MotionManager::GetInstance()->AddModule((MotionModule*) Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Walking::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Kicking::GetInstance());

    LinuxMotionTimer motion_timer(MotionManager::GetInstance());
    motion_timer.Start();
    /////////////////////////////////////////////////////////////////////

    MotionManager::GetInstance()->LoadINISettings(&ini);
    StateMachine::GetInstance()->LoadINISettings(&ini);

    int firm_ver = 0;
    if (cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, nullptr) != CM730::SUCCESS) {
        LOG_FATAL << "Can't read firmware version from Dynamixel ID " << JointData::ID_HEAD_PAN;
        return 1;
    }

    if (0 < firm_ver && firm_ver < 27) {
        LOG_FATAL << "MX-28's firmware is not support 4096 resolution! "
                  << "Upgrade MX-28's firmware to version 27(0x1B) or higher.";
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
//    LOG_INFO << "Game controller client is ready";

    /////////////////////////////////////////////////////////////////////

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, nullptr);

    SoccerBehavior soccer;
    GoalieBehavior goalie;

    soccer.LoadINISettings(&ini);
    goalie.LoadINISettings(&ini);

//    while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);

    LOG_INFO << "=== Initialization was finished ===";
    LOG_INFO << "=== Controller was started ===";
//    Action::GetInstance()->Start(9);
    while (!finish) {
        // Update game controller
        GameController::GetInstance()->Update();

        // Update state machine
        StateMachine::GetInstance()->Check(&cm730);

        if (!Action::GetInstance()->IsRunning() && !Kicking::GetInstance()->IsRunning() && !Walking::GetInstance()->IsRunning()) {
//            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
//            Walking::GetInstance()->SetXMoveAmplitude(10);
//            Walking::GetInstance()->Start();
            Kicking::GetInstance()->SetShiftingBodyDuration(300);
            Kicking::GetInstance()->SetKickingDuration(150);
            Kicking::GetInstance()->SetRestoringDuration(200);
            Kicking::GetInstance()->SetKickingLeg(Kicking::RIGHT_LEG);
            Kicking::GetInstance()->SetKickTargetXOffset(35.0);
            Kicking::GetInstance()->SetKickYOffset(-2.0);
            Kicking::GetInstance()->SetKickXOffset(0.0);
            Kicking::GetInstance()->SetKickYawOffset(radians(-15));
//            Kicking::GetInstance()->SetBodyInitPitchOffset(radians(13.0f));
            Kicking::GetInstance()->SetBodyInitXOffset(0.0);
//            Kicking::GetInstance()->SetBodyInitZOffset(20.0f);
            Kicking::GetInstance()->SetBodyXOffset(0.0);
//            Kicking::GetInstance()->SetBodyZOffset(20.0f);
//            Kicking::GetInstance()->SetKickZOffset(0.0);
            Kicking::GetInstance()->Kick();
            Kicking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
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
//            case ROLES_COUNT:break;
//        }
    }
    LOG_INFO << "=== Controller was terminated ===";


    return 0;
}
