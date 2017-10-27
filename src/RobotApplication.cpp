#include "log/Logger.h"
#include "motion/MotionManager.h"
#include "gamecontroller/GameController.h"
#include "motion/modules/Action.h"
#include "motion/modules/Kicking.h"
#include "motion/modules/Walking.h"
#include "motion/modules/Head.h"
#include "config/CommandArguments.h"
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
    m_arguments = arguments;
}

int RobotApplication::Exec() {
    ParseCommandLineArguments();
    if (!TryStart()) return EXEC_FAILED;
    try {
        Initialize();
        StartMainLoop();
        return EXEC_SUCCESS;
    } catch(const std::exception& e) {
        LOG_FATAL << "Application terminated: " << e.what();
        return EXEC_FAILED;
    }
}

bool RobotApplication::HelpMessageRequested() const noexcept {
    return m_show_help_message;
}

bool RobotApplication::TryStart() {
    if (HelpMessageRequested()) {
        return false;
    }
    if (IsRunning()) {
        if (m_debug) LOG_WARNING << "Robot application already started";
        return false;
    }
    m_started.store(true, std::memory_order_release);
    return true;
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
    //TODO Don't forget uncomment this lines
    m_configuration_loader.SetDefaultPath(m_config_default);

//    m_configuration_loader.AddStrategy(m_ball_searcher_configuration_strategy, m_config_ball_searcher);
//    m_configuration_loader.AddStrategy(m_ball_tracker_configuration_strategy, m_config_ball_searcher);
    m_configuration_loader.AddStrategy(m_game_controller_configuration_strategy, m_config_game_controller);
    m_configuration_loader.AddStrategy(m_head_configuration_strategy, m_config_head);
    m_configuration_loader.AddStrategy(m_walking_configuration_strategy, m_config_walking);
    m_configuration_loader.AddStrategy(m_motion_manager_configuration_strategy, m_config_motion_manager);

    m_action_configuration_loader.SetPath(m_config_action);
    if (m_debug) LOG_INFO << "Configuration loader is ready";
}

void RobotApplication::ParseCommandLineArguments() {
    namespace po = boost::program_options;

    po::options_description desk("Allowed options");

    bool help_requested = false;

    bool debug_all = false;
    bool debug_application = false;
    bool debug_ball_searcher = false;
    bool debug_ball_tracker = false;
    bool debug_game_controller = false;
    bool debug_motion_manager = false;
    bool debug_head = false;
    bool debug_walking = false;
    bool debug_action = false;
    bool debug_kicking = false;

    std::string config_default;
    std::string config_ball_searcher;
    std::string config_ball_tracker;
    std::string config_game_controller;
    std::string config_motion_manager;
    std::string config_head;
    std::string config_walking;
    std::string config_action;
    std::string config_kicking;

    desk.add_options()
            ("help,h", po::value(&help_requested)->zero_tokens(), "produce help message")
            // Debug output
            ("dbg-all,d", po::value(&debug_all)->zero_tokens(), "enable debug output for all components")
            ("dbg-app", po::value(&debug_application)->zero_tokens(), "enable debug output for main application")
            ("dbg-ball-searcher", po::value(&debug_ball_searcher)->zero_tokens(), "enable debug output for ball searcher")
            ("dbg-ball-tracker", po::value(&debug_ball_tracker)->zero_tokens(), "enable debug output for ball tracker")
            ("dbg-game-contoller", po::value(&debug_game_controller)->zero_tokens(), "enable debug output for game controller")
            ("dbg-motion-manager", po::value(&debug_motion_manager)->zero_tokens(), "enable debug output for motion manager")
            ("dbg-head", po::value(&debug_head)->zero_tokens(), "enable debug output for head motion module")
            ("dbg-walking", po::value(&debug_walking)->zero_tokens(), "enable debug output for walking motion module")
            ("dbg-action", po::value(&debug_action)->zero_tokens(), "enable debug output for action motion module")
            ("dbg-kicking", po::value(&debug_kicking)->zero_tokens(), "enable debug output for kicking motion module")
            // Config files
            ("cfg,c", po::value(&config_default)->value_name("path"), "default config file (res/config.ini by default)")
            ("cfg-ball-tracker", po::value(&config_ball_searcher)->value_name("path"), "config file for ball tracker")
            ("cfg-ball-searcher", po::value(&config_ball_tracker)->value_name("path"), "config file for ball searcher")
            ("cfg-game-controller", po::value(&config_game_controller)->value_name("path"), "config file for game controller")
            ("cfg-motion-manager", po::value(&config_motion_manager)->value_name("path"), "config file for motion manager")
            ("cfg-head", po::value(&config_head)->value_name("path"), "config file for head motion module")
            ("cfg-walking", po::value(&config_walking)->value_name("path"), "config file for walking motion module")
            ("cfg-kicking", po::value(&config_kicking)->value_name("path"), "config file for kicking motion module")
            ("cfg-action", po::value(&config_action)->value_name("path"), "path to motion_4096.bin");

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(m_arguments.GetArgc(), m_arguments.GetArgv())
                          .options(desk)
                          .allow_unregistered()
                          .run(), vm);
        po::notify(vm);
    } catch (const boost::program_options::error& e) {
        std::cout << e.what() << std::endl;
        help_requested = true;
    }

    if (help_requested) {
        m_show_help_message = help_requested;
        std::cout << desk << std::endl;
    } else {
        if (debug_all || debug_application) EnableDebug(true);
        if (debug_all || debug_ball_searcher) ;// TODO Enable debug for ball searcher
        if (debug_all || debug_ball_tracker) ;// TODO Enable debug for ball tracker
        if (debug_all || debug_game_controller) ;// TODO Enable debug for ball tracker
        if (debug_all || debug_motion_manager) MotionManager::GetInstance()->EnabledDebug(true);
        if (debug_all || debug_head) ; // TODO Enable debug for head
        if (debug_all || debug_walking) ; // TODO Enable debug for walking
        if (debug_all || debug_action) Action::GetInstance()->EnableDebug(true);
        if (debug_all || debug_kicking) Kicking::GetInstance()->EnableDebug(true);

        if (!config_default.empty()) m_config_default = config_default;
        if (!config_ball_searcher.empty()) m_config_ball_searcher = config_ball_searcher;
        if (!config_ball_tracker.empty()) m_config_ball_tracker = config_ball_tracker;
        if (!config_game_controller.empty()) m_config_game_controller = config_game_controller;
        if (!config_motion_manager.empty()) m_config_motion_manager = config_motion_manager;
        if (!config_head.empty()) m_config_head = config_head;
        if (!config_walking.empty()) m_config_walking = config_walking;
        if (!config_action.empty()) m_config_action = config_action;
        if (!config_kicking.empty()) m_config_kicking = config_kicking;
    }
}

void RobotApplication::ReadConfiguration() {
    if (m_debug) LOG_DEBUG << "Reading configuration...";
    m_configuration_loader.ConfigureAll();
    m_action_configuration_loader.ReadMotionFile();
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
