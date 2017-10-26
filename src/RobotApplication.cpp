#include <log/Logger.h>
#include <motion/MotionManager.h>
#include <gamecontroller/GameController.h>
#include <motion/modules/Action.h>
#include <motion/modules/Kicking.h>
#include <motion/modules/Walking.h>
#include <motion/modules/Head.h>
#include <tool/CommandArguments.h>
#include "RobotApplication.h"
#ifdef CROSSCOMPILE
	#include "hw/RobotCM730.h"
#else
	#include "hw/VrepCM730.h"
#endif

using namespace Robot;


RobotApplication* RobotApplication::GetInstance() {
    static RobotApplication instance;
    return &instance;
}

void RobotApplication::SetProgramArguments(const CommandArguments& arguments) {
    if (m_debug) LOG_DEBUG << "=== Parsing command line arguments ===";
}

int RobotApplication::Exec() {
    if (TryStart()) return EXEC_FAILED;
    try {
        Initialize();
        StartMainLoop();
        return EXEC_SUCCESS;
    } catch(const std::exception& e) {
        LOG_FATAL << "Application terminated: " << e.what();
        return EXEC_FAILED;
    }
}

bool RobotApplication::TryStart() {
    if (IsRunning()) {
        if (m_debug) LOG_WARNING << "Robot application already started";
        return true;
    }
    m_started.store(true, std::memory_order_release);
    return false;
}

void RobotApplication::Stop() {
    m_started.store(false, std::memory_order_release);
}

bool RobotApplication::IsRunning() const {
    return m_started.load(std::memory_order_acquire);
}

void RobotApplication::Initialize() {
    if (m_debug) LOG_INFO << "=== Initialization was started ===";
    CheckHWStatus();
    InitCM730();
    InitMotionManager();
    InitMotionModules();
    InitMotionTimer();
    InitGameController();
    InitConfiguraionLoader();
    ParseCommandLineArguments();
    ReadConfiguration();
    if (m_debug) LOG_INFO << "=== Initialization was finished ===";
}

void RobotApplication::CheckHWStatus() {
    if (m_cm730 != nullptr) {
        throw std::runtime_error("Hardware already was initialized");
    }
}

void RobotApplication::InitCM730() {
    if (m_debug) LOG_DEBUG << "Initializing hardware...";
#ifdef CROSSCOMPILE
    auto linux_cm730 = std::make_unique<LinuxCM730>("/dev/ttyUSB0"); //Undefined reference to U2D_DEV_NAME0
    auto cm730 = std::make_unique<RobotCM730>(linux_cm730.get());
    m_linux_cm730 = std::move(linux_cm730);
    m_cm730 = std::move(cm730);
#else
    auto vrep_connector{std::make_unique<VrepConnector>()};
    auto vrep_cm730{std::make_unique<VrepCM730>()};
    vrep_connector->Connect();
    vrep_cm730->SetClientId(vrep_connector->GetClientID());
    vrep_cm730->Connect();
    m_vrep_connector = move(vrep_connector);
    m_cm730 = move(vrep_cm730);
#endif
    if (m_debug) LOG_INFO << "Hardware is ready";
}

void RobotApplication::CheckFirmware() {
    int firm_ver = 0;
    auto read_result = m_cm730->ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, nullptr);
	LOG_DEBUG << "Firmware version: " << firm_ver;
    if (read_result != CM730::SUCCESS) {
        throw std::runtime_error("Can't read firmware version from Dynamixel ID " +
                                         std::to_string(JointData::ID_HEAD_PAN));
    }

    if (0 < firm_ver && firm_ver < 27) {
        throw std::runtime_error("MX-28's firmware is not support 4096 resolution! "
                                 "Upgrade MX-28's firmware to version 27(0x1B) or higher.");
    } else if (27 <= firm_ver) {
        // Do nothing
    } else {
        throw std::runtime_error("Unknown version of MX-28's firmware");
    }
}

void RobotApplication::InitMotionManager() {
    if (m_debug) LOG_DEBUG << "Initializing motion manager...";
    if (!MotionManager::GetInstance()->Initialize(m_cm730.get())) {
        throw std::runtime_error("Fail to initialize Motion Manager!");
    }
    if (m_debug) LOG_INFO << "Motion manager is ready";
}

void RobotApplication::InitMotionModules() {
    if (m_debug) LOG_DEBUG << "Initializing motion modules...";
    MotionManager::GetInstance()->AddModule((MotionModule*) Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Walking::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*) Kicking::GetInstance());
    if (m_debug) LOG_INFO << "Motion modules are ready";
}

void RobotApplication::InitMotionTimer() {
    if (m_debug) LOG_DEBUG << "Initializing motion timer...";
    auto motion_timer = std::make_unique<LinuxMotionTimer>(MotionManager::GetInstance());
    motion_timer->Start();
    m_motion_timer = std::move(motion_timer);
    if (m_debug) LOG_INFO << "Motion timer is ready";
    CheckFirmware(); //This was moved here from Cm730 inititialization
}

void RobotApplication::InitGameController() {
    if (m_debug) LOG_DEBUG << "Initializing game controller client...";
    if (!GameController::GetInstance()->Connect()) {
        throw std::runtime_error("Can't connect to game controller!");
    }
    if (m_debug) LOG_INFO << "Game controller client is ready";
}

void RobotApplication::InitConfiguraionLoader() {
    if (m_debug) LOG_DEBUG << "Initializing configuration loader...";
    m_configuration_loader.AddStrategy(m_walking_configuration_strategy, std::__cxx11::string());
    m_configuration_loader.AddStrategy(m_motion_manager_configuration_strategy, std::__cxx11::string());
    if (m_debug) LOG_INFO << "Configuration loader is ready";
}

void RobotApplication::ParseCommandLineArguments() {
    if (m_debug) LOG_DEBUG << "Parsing command line arguments...";
    if (m_debug) LOG_INFO << "Command line arguments was parsed";
}

void RobotApplication::ReadConfiguration() {
    if (m_debug) LOG_DEBUG << "Reading configuration...";
    m_configuration_loader.ConfigureAll();
    if (m_debug) LOG_INFO << "Reading configuration was finished";
}

void RobotApplication::StartMainLoop() {
    if (m_debug) LOG_INFO << "=== Controller was started ===";
    while (IsRunning()) {

    }
    if (m_debug) LOG_INFO << "=== Contoller was finished ===";

}

bool RobotApplication::IsDebug() const noexcept {
    return m_debug;
}

void RobotApplication::EnableDebug(bool debug) noexcept {
    m_debug = debug;
}
