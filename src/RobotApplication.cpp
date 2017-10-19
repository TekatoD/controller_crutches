#include <log/Logger.h>
#include <motion/MotionManager.h>
#include <GameController.h>
#include "RobotApplication.h"
#include "hw/VrepCM730.h"

using namespace Robot;


RobotApplication* RobotApplication::GetInstance() {
    static RobotApplication instance;
    return &instance;
}

void RobotApplication::ParseArguments(int argc, const char** argv) {
    if (m_debug) LOG_DEBUG << "=== Parsing command line arguments ===";
}

int RobotApplication::Start() {
    // If application already started then break
    if (m_started.load(std::memory_order_acquire)) {
        if (m_debug) LOG_WARNING << "Robot application already started";
        return 1;
    }
}

void RobotApplication::Stop() {
    m_started.store(false, std::memory_order_release);
}

void RobotApplication::Initialize() {
    if (m_debug) LOG_INFO << "=== Initialization was started ===";
    CheckHWStatus();
    InitCM730();
    InitMotionManager();
    InitMotionTimer();
    InitGameController();
    if (m_debug) LOG_INFO << "=== Initialization was finished ===";
}

void RobotApplication::CheckHWStatus() {
    if (m_cm730 != nullptr) {
        constexpr char msg[] = "Hardware already was initialized";
        LOG_FATAL << msg;
        throw std::runtime_error(msg);
    }
}

void RobotApplication::InitCM730() {
    if (m_debug) LOG_DEBUG << "Initializing hardware...";
#ifdef CROSSCOMPILE
    auto linux_cm730{std::make_unique<LinuxCM730>(U2D_DEV_NAME0)};
    auto cm730{std::make_unique<RobotCM730>(cm730.get())};
    m_linux_cm730{std::move(linux_cm730)};
    m_cm730{std::move(cm730)};
#else
    auto vrep_connector{std::make_unique<VrepConnector>()};
    auto vrep_cm730{std::make_unique<VrepCM730>()};
    vrep_connector->Connect();
    vrep_cm730->SetClientId(vrep_connector->GetClientID());
    vrep_cm730->Connect();
    m_vrep_connector = move(vrep_connector);
    m_cm730 = move(vrep_cm730);
#endif
    CheckFirmware();

    if (m_debug) LOG_INFO << "Hardware is ready";
}

void RobotApplication::InitMotionManager() {
    if (m_debug) LOG_DEBUG << "Initializing motion manager...";
    if (!MotionManager::GetInstance()->Initialize(m_cm730.get())) {
        constexpr char msg[] = "Fail to initialize Motion Manager!";
        LOG_FATAL << msg;
        throw std::runtime_error(msg);
    }
    if (m_debug) LOG_INFO << "Motion manager is ready";
}

void RobotApplication::InitMotionTimer() {
    if (m_debug) LOG_DEBUG << "Initializing motion timer...";
    auto motion_timer = std::make_unique<LinuxMotionTimer>(MotionManager::GetInstance());
    motion_timer->Start();
    m_motion_timer = std::move(motion_timer);
    if (m_debug) LOG_INFO << "Motion timer is ready";
}

bool RobotApplication::IsDebug() const noexcept {
    return m_debug;
}

void RobotApplication::EnableDebug(bool debug) noexcept {
    m_debug = debug;
}

void RobotApplication::InitGameController() {
    if (m_debug) LOG_DEBUG << "Initializing game controller client...";
    if (!GameController::GetInstance()->Connect()) {
        constexpr char msg[]{"Can't connect to game controller!"};
        LOG_ERROR << msg;
        throw std::runtime_error(msg);
    }
    if (m_debug) LOG_INFO << "Game controller client is ready";
}

void RobotApplication::ReadConfiguration() {
    if (m_debug) LOG_DEBUG << "=== Read configurations ===";
}

void RobotApplication::CheckFirmware() {
    int firm_ver = 0;
    if (m_cm730->ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, nullptr) != CM730::SUCCESS) {
        auto msg{"Can't read firmware version from Dynamixel ID " + std::to_string(JointData::ID_HEAD_PAN)};
        LOG_FATAL << msg;
        throw std::runtime_error(msg);
    }

    if (0 < firm_ver && firm_ver < 27) {
        constexpr char msg[]{
                "MX-28's firmware is not support 4096 resolution! "
                        "Upgrade MX-28's firmware to version 27(0x1B) or higher."
        };
        LOG_FATAL << msg;
        throw std::runtime_error(msg);
    } else if (27 <= firm_ver) {
        // Do nothing
    } else {
        throw std::runtime_error("Unknown version of MX-28's firmware");
    }
}
