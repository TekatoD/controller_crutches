#include <config/ArgumentsParser.h>
#include <hw/buttons_t.h>
#include <hw/LEDs_t.h>
#include "log/trivial_logger_t.h"
#include "motion/motion_manager_t.h"
#include "gamecontroller/GameController.h"
#include "motion/modules/action_t.h"
#include "motion/modules/kicking_t.h"
#include "motion/modules/walking_t.h"
#include "motion/modules/head_t.h"
#include "robot_application_t.h"

#ifdef CROSSCOMPILE
#include "hw/robot_CM730_t.h"
#else

#include "hw/vrep_CM730_t.h"

#endif

using namespace drwn;


robot_application_t* robot_application_t::GetInstance() {
    static robot_application_t instance;
    return &instance;
}

void robot_application_t::set_program_arguments(const CommandArguments& arguments) {
    m_arguments = arguments;
}

int robot_application_t::exec() {
    parse_command_line_arguments();
    if (!try_start()) return EXEC_FAILED;
    try {
        initialize();
        start_main_loop();
        return EXEC_SUCCESS;
    } catch (const std::exception& e) {
        LOG_FATAL << "Application terminated: " << e.what();
        return EXEC_FAILED;
    }
}

bool robot_application_t::help_message_requested() const noexcept {
    return m_arg_help_requested.IsHelpRequested();
}

bool robot_application_t::try_start() {
    if (help_message_requested()) {
        return false;
    }
    if (is_running()) {
        if (m_debug) LOG_WARNING << "Robot application already started";
        return false;
    }
    m_started.store(true, std::memory_order_release);
    return true;
}

void robot_application_t::stop() {
    m_started.store(false, std::memory_order_release);
}

bool robot_application_t::is_running() const {
    return m_started.load(std::memory_order_acquire);
}

void robot_application_t::initialize() {
    apply_debug_arguments();
    if (m_debug) LOG_INFO << "=== Initialization was started ===";
    check_hw_status();
    init_CM730();
    init_motion_manager();
    init_motion_modules();
    init_motion_timer();
    init_game_controller();
    init_configuraion_loader();
    read_configuration();
    if (m_debug) LOG_INFO << "=== Initialization was finished ===";
}

void robot_application_t::check_hw_status() {
    if (m_cm730 != nullptr) {
        throw std::runtime_error("Hardware already was initialized");
    }
}

void robot_application_t::init_CM730() {
    if (m_debug) LOG_DEBUG << "Initializing hardware...";
#ifdef CROSSCOMPILE
    auto linux_cm730 = std::make_unique<linux_CM730_t>("/dev/ttyUSB0"); //Undefined reference to U2D_DEV_NAME0
    auto cm730 = std::make_unique<robot_CM730_t>(linux_cm730.get());
    m_linux_cm730 = std::move(linux_cm730);
    m_cm730 = std::move(cm730);
#else
    auto vrep_connector{std::make_unique<vrep_connector_t>()};
    auto vrep_cm730{std::make_unique<vrep_CM730_t>()};
    vrep_connector->Connect();
    vrep_cm730->SetClientId(vrep_connector->GetClientID());
    vrep_cm730->Connect();
    m_vrep_connector = move(vrep_connector);
    m_cm730 = move(vrep_cm730);
#endif

    LEDs_t::GetInstance()->initialize(m_cm730.get());
    if (m_debug) LOG_INFO << "Hardware is ready";
}

void robot_application_t::check_firmware() {
    int firm_ver = 0;
    auto read_result = m_cm730->ReadByte(joint_data_t::ID_HEAD_PAN, MX28_t::P_VERSION, &firm_ver, nullptr);
    if (m_debug) LOG_DEBUG << "Firmware version: " << firm_ver;
    if (read_result != CM730_t::SUCCESS) {
        throw std::runtime_error("Can't read firmware version from Dynamixel ID " +
                                 std::to_string(joint_data_t::ID_HEAD_PAN));
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

void robot_application_t::init_motion_manager() {
    if (m_debug) LOG_DEBUG << "Initializing motion manager...";
    if (!motion_manager_t::GetInstance()->initialize(m_cm730.get())) {
        throw std::runtime_error("Fail to initialize Motion Manager!");
    }
    if (m_debug) LOG_INFO << "Motion manager is ready";
}

void robot_application_t::init_motion_modules() {
    if (m_debug) LOG_DEBUG << "Initializing motion modules...";
    motion_manager_t::GetInstance()->add_module((motion_module_t*) action_t::GetInstance());
    motion_manager_t::GetInstance()->add_module((motion_module_t*) head_t::GetInstance());
    motion_manager_t::GetInstance()->add_module((motion_module_t*) walking_t::GetInstance());
    motion_manager_t::GetInstance()->add_module((motion_module_t*) kicking_t::GetInstance());
    if (m_debug) LOG_INFO << "Motion modules are ready";
}

void robot_application_t::init_motion_timer() {
    if (m_debug) LOG_DEBUG << "Initializing motion timer...";
    auto motion_timer = std::make_unique<linux_motion_timer_t>(motion_manager_t::GetInstance());
    motion_timer->Start();
    m_motion_timer = std::move(motion_timer);
    if (m_debug) LOG_INFO << "Motion timer is ready";
    check_firmware(); //This was moved here from Cm730 inititialization
}

void robot_application_t::init_game_controller() {
    if (m_debug) LOG_DEBUG << "Initializing game controller client...";
    if (!GameController::GetInstance()->Connect()) {
        throw std::runtime_error("Can't connect to game controller!");
    }
    if (m_debug) LOG_INFO << "Game controller client is ready";
}

void robot_application_t::init_configuraion_loader() {
    if (m_debug) LOG_DEBUG << "Initializing configuration loader...";
    //TODO Don't forget uncomment this lines
    m_configuration_loader.SetDefaultPath(m_arg_config_default);

//    m_configuration_loader.AddStrategy(m_ball_searcher_configuration_strategy, m_arg_config_ball_searcher);
//    m_configuration_loader.AddStrategy(m_ball_tracker_configuration_strategy, m_arg_config_ball_searcher);
    m_configuration_loader.AddStrategy(m_game_controller_configuration_strategy, m_arg_config_game_controller);
    m_configuration_loader.AddStrategy(m_head_configuration_strategy, m_arg_config_head);
    m_configuration_loader.AddStrategy(m_walking_configuration_strategy, m_arg_config_walking);
    m_configuration_loader.AddStrategy(m_motion_manager_configuration_strategy, m_arg_config_motion_manager);
    // Action configuration loader uses self loader
    m_action_configuration_loader.SetPath(m_arg_config_action);
    if (m_debug) LOG_INFO << "Configuration loader is ready";
}

void robot_application_t::parse_command_line_arguments() {
    namespace po = boost::program_options;

    ArgumentsParser parser("Allowed options");
    parser.SetArguments(m_arguments);

    parser.AddStrategy(m_arg_help_requested);

    parser.AddStrategy(m_arg_debug_all);
    parser.AddStrategy(m_arg_debug_application);
    parser.AddStrategy(m_arg_debug_ball_searcher);
    parser.AddStrategy(m_arg_debug_ball_tracker);
    parser.AddStrategy(m_arg_debug_game_controller);
    parser.AddStrategy(m_arg_debug_motion_manager);
    parser.AddStrategy(m_arg_debug_head);
    parser.AddStrategy(m_arg_debug_walking);
    parser.AddStrategy(m_arg_debug_action);
    parser.AddStrategy(m_arg_debug_kicking);
    parser.AddStrategy(m_arg_debug_buttons);
    parser.AddStrategy(m_arg_debug_leds);

    parser.AddStrategy(m_arg_config_default);
    parser.AddStrategy(m_arg_config_ball_searcher);
    parser.AddStrategy(m_arg_config_ball_tracker);
    parser.AddStrategy(m_arg_config_game_controller);
    parser.AddStrategy(m_arg_config_motion_manager);
    parser.AddStrategy(m_arg_config_head);
    parser.AddStrategy(m_arg_config_walking);
    parser.AddStrategy(m_arg_config_action);
    parser.AddStrategy(m_arg_config_kicking);

    m_arg_help_requested.SetOption("help,h", "produce help message");

    m_arg_debug_all.SetOption("dbg-all,d", "enable debug output for all components");
    m_arg_debug_application.SetOption("dbg-app", "enable debug output for main application");
    m_arg_debug_ball_searcher.SetOption("dbg-ball-searcher", "enable debug output for ball searcher");
    m_arg_debug_ball_tracker.SetOption("dbg-ball-tracker", "enable debug output for ball tracker");
    m_arg_debug_game_controller.SetOption("dbg-game-contoller", "enable debug output for game controller");
    m_arg_debug_motion_manager.SetOption("dbg-motion-manager", "enable debug output for motion manager");
    m_arg_debug_head.SetOption("dbg-head", "enable debug output for head motion module");
    m_arg_debug_walking.SetOption("dbg-walking", "enable debug output for walking motion module");
    m_arg_debug_action.SetOption("dbg-action", "enable debug output for action motion module");
    m_arg_debug_kicking.SetOption("dbg-kicking", "enable debug output for kicking motion module");
    m_arg_debug_buttons.SetOption("dbg-kicking", "enable debug output for buttons");
    m_arg_debug_leds.SetOption("dbg-kicking", "enable debug output for LEDs");

    m_arg_config_default.SetOption("cfg,c", "default config file (res/config.ini by default)");
    m_arg_config_ball_tracker.SetOption("cfg-ball-tracker", "config file for ball tracker");
    m_arg_config_ball_searcher.SetOption("cfg-ball-searcher", "config file for ball searcher");
    m_arg_config_game_controller.SetOption("cfg-game-controller", "config file for game controller");
    m_arg_config_motion_manager.SetOption("cfg-motion-manager", "config file for motion manager");
    m_arg_config_head.SetOption("cfg-head", "config file for head motion module");
    m_arg_config_walking.SetOption("cfg-walking", "config file for walking motion module");
    m_arg_config_kicking.SetOption("cfg-kicking", "config file for kicking motion module");
    m_arg_config_action.SetOption("cfg-action", "path to motion_4096.bin");

    if (!parser.Parse() || m_arg_help_requested.IsHelpRequested()) {
        parser.ShowDescriptionToStream(std::cout);
    }
}

void robot_application_t::apply_debug_arguments() {
    if (m_arg_debug_all || m_arg_debug_application)
        enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_ball_searcher)
        ; // TODO Enable debug for ball searcher
    if (m_arg_debug_all || m_arg_debug_ball_tracker)
        ; // TODO Setting debug for ball tracker
    if (m_arg_debug_all || m_arg_debug_game_controller)
        GameController::GetInstance()->EnableDebug(true);
    if (m_arg_debug_all || m_arg_debug_motion_manager)
        motion_manager_t::GetInstance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_head)
        head_t::GetInstance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_walking)
        walking_t::GetInstance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_action)
        action_t::GetInstance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_kicking)
        kicking_t::GetInstance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_buttons)
        buttons_t::get_instance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_leds)
        LEDs_t::GetInstance()->enable_debug(true);
}

void robot_application_t::read_configuration() {
    if (m_debug) LOG_DEBUG << "Reading configuration...";
    m_configuration_loader.ConfigureAll();
    m_action_configuration_loader.ReadMotionFile();
    if (m_debug) LOG_INFO << "Reading configuration was finished";
}

void robot_application_t::start_main_loop() {
    if (m_debug) LOG_INFO << "=== Controller was started ===";
    while (is_running()) {

    }
    if (m_debug) LOG_INFO << "=== Contoller was finished ===";

}

bool robot_application_t::is_debug() const noexcept {
    return m_debug;
}

void robot_application_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}
