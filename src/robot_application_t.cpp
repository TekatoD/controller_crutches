#include <iostream>
#include <config/arguments_parser_t.h>
#include <hw/buttons_t.h>
#include <hw/LEDs_t.h>
#include <hw/vrep_image_source_t.h>
#include <hw/camera_t.h>
#include <hw/image_source_failure.h>
#include <opencv/cv.hpp>
#include <vision/vision_t.h>
#include "log/trivial_logger_t.h"
#include "motion/motion_manager_t.h"
#include "gamecontroller/game_controller_t.h"
#include "motion/modules/action_t.h"
#include "motion/modules/kicking_t.h"
#include "motion/modules/walking_t.h"
#include "motion/modules/head_t.h"
#include "localization/particle_filter_t.h"
#include "localization/localization_t.h"
#include "robot_application_t.h"

#ifdef CROSSCOMPILE
#include "hw/robot_CM730_t.h"
#else
#include "hw/vrep_CM730_t.h"
#endif

using namespace drwn;


robot_application_t* robot_application_t::get_instance() {
    static robot_application_t instance;
    return &instance;
}

void robot_application_t::set_program_arguments(const command_arguments_t& arguments) {
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
    return m_arg_help_requested.is_help_requested();
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
    this->apply_debug_arguments();
    if (m_debug) LOG_INFO << "=== Initialization was started ===";
    this->check_hw_status();
    this->init_CM730();
    this->init_cv();
    this->init_localization();
    this->init_motion_manager();
    this->init_motion_modules();
    this->init_motion_timer();
    this->init_game_controller();
    this->init_configuraion_loader();
    this->read_configuration();
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
    auto vrep_image_source{std::make_unique<vrep_image_source_t>()};
    vrep_connector->connect();
    vrep_cm730->set_client_id(vrep_connector->get_client_id());
    vrep_cm730->connect();
    m_vrep_connector = move(vrep_connector);
    m_cm730 = move(vrep_cm730);
    camera_t::get_instance()->set_image_source(m_image_source.get());

#endif

    LEDs_t::GetInstance()->initialize(m_cm730.get());
    if (m_debug) LOG_INFO << "Hardware is ready";
}

void robot_application_t::check_firmware() {
    int firm_ver = 0;
    auto read_result = m_cm730->read_byte(joint_data_t::ID_HEAD_PAN, MX28_t::P_VERSION, &firm_ver, nullptr);
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

void robot_application_t::init_cv() {
    if (m_debug) LOG_DEBUG << "Initializing camera...";
#ifdef CROSSCOMPILATION
    auto image_source = std::make_unique<robot_image_source_t>(camera_t::WIDTH, camera_t::HEIGHT);
#else
    auto image_source = std::make_unique<vrep_image_source_t>("camera", camera_t::WIDTH, camera_t::HEIGHT);
    image_source->set_client_id(m_vrep_connector->get_client_id());
    image_source->connect();
#endif
    m_image_source = std::move(image_source);
    camera_t::get_instance()->set_image_source(m_image_source.get());
    if (m_arg_debug_all || m_arg_debug_image_source)
        m_image_source->enable_debug(true);
    if (m_debug) LOG_INFO << "Camera is ready";

    if (m_debug) LOG_DEBUG << "Initializing CV pipeline...";
    auto vision_processor = std::make_unique<white_ball_vision_processor_t>();
    const std::string& path = m_arg_white_ball_vision_processor.get_dump_images_path();
    if(!path.empty()) {
        vision_processor->set_dump_directory_path(path);
    }
    vision_processor->enable_dump_images(m_arg_white_ball_vision_processor.is_dump_images_enabled());
    vision_processor->enable_show_images(m_arg_white_ball_vision_processor.is_display_images_enabled());
    m_vision_processor = std::move(vision_processor);
    vision_t::get_instance()->set_processor(m_vision_processor.get());
    if (m_arg_debug_all || m_arg_debug_vision_processor)
        m_vision_processor->enable_debug(true);
    if (m_debug) LOG_INFO << "CV pipeline is ready";
}

void robot_application_t::init_localization() {
    if (m_debug) LOG_DEBUG << "Initializing localization...";

    auto particle_filter = std::make_unique<particle_filter_t>();
    m_particle_filter = std::move(particle_filter);

    localization_t::get_instance()->set_particle_filter(m_particle_filter.get());
    if (m_arg_debug_all || m_arg_debug_localization) {
        localization_t::get_instance()->enable_debug(true);
    }

    if (m_debug) LOG_INFO << "Localization is ready";
}

void robot_application_t::init_motion_manager() {
    if (m_debug) LOG_DEBUG << "Initializing motion manager...";
    if (!motion_manager_t::get_instance()->initialize(m_cm730.get())) {
        throw std::runtime_error("Fail to initialize Motion Manager!");
    }
    if (m_debug) LOG_INFO << "Motion manager is ready";
}


void robot_application_t::init_motion_modules() {
    if (m_debug) LOG_DEBUG << "Initializing motion modules...";
    motion_manager_t::get_instance()->add_module((motion_module_t*) action_t::get_instance());
    motion_manager_t::get_instance()->add_module((motion_module_t*) head_t::get_instance());
    motion_manager_t::get_instance()->add_module((motion_module_t*) walking_t::get_instance());
    motion_manager_t::get_instance()->add_module((motion_module_t*) kicking_t::get_instance());
    if (m_debug) LOG_INFO << "Motion modules are ready";
}

void robot_application_t::init_motion_timer() {
    if (m_debug) LOG_DEBUG << "Initializing motion timer...";
    auto motion_timer = std::make_unique<linux_motion_timer_t>(motion_manager_t::get_instance());
    motion_timer->start();
    m_motion_timer = std::move(motion_timer);
    if (m_debug) LOG_INFO << "Motion timer is ready";
    check_firmware(); //This was moved here from Cm730 inititialization
}

void robot_application_t::init_game_controller() {
    if (m_debug) LOG_DEBUG << "Initializing game controller client...";
    if (!game_controller_t::get_instance()->connect()) {
        throw std::runtime_error("Can't connect to game controller!");
    }
    if (m_debug) LOG_INFO << "Game controller client is ready";
}

void robot_application_t::init_configuraion_loader() {
    if (m_debug) LOG_DEBUG << "Initializing configuration loader...";
    //TODO Don't forget uncomment this lines
    m_white_ball_vision_processor_configuration_strategy.set_white_ball_vision_processor(m_vision_processor.get());
    m_localization_field_configuration_strategy.set_field_map(m_particle_filter->get_field_map());
    m_particle_filter_configuration_strategy.set_particle_filter(m_particle_filter.get());

    m_configuration_loader.set_default_path(m_arg_config_default);
//    m_configuration_loader.add_strategy(m_ball_searcher_configuration_strategy, m_arg_config_ball_searcher);
//    m_configuration_loader.add_strategy(m_ball_tracker_configuration_strategy, m_arg_config_ball_searcher);
    m_configuration_loader.add_strategy(m_game_controller_configuration_strategy, m_arg_config_game_controller);
    m_configuration_loader.add_strategy(m_head_configuration_strategy, m_arg_config_head);
    m_configuration_loader.add_strategy(m_walking_configuration_strategy, m_arg_config_walking);
    m_configuration_loader.add_strategy(m_motion_manager_configuration_strategy, m_arg_config_motion_manager);
    m_configuration_loader.add_strategy(m_white_ball_vision_processor_configuration_strategy, m_arg_config_white_ball_vision_processor);
    m_configuration_loader.add_strategy(m_localization_field_configuration_strategy, m_arg_config_localization_field);
    m_configuration_loader.add_strategy(m_particle_filter_configuration_strategy, m_arg_config_particle_filter);

#ifdef CROSSCOMPILATION
    m_robot_image_source_configuration_strategy.set_image_source(m_image_source.get());
    m_configuration_loader.add_strategy(m_robot_image_source_configuration_strategy, m_arg_config_image_source);
#endif
    // Action configuration loader uses self loader
    m_action_configuration_loader.set_path(m_arg_config_action);
    if (m_debug) LOG_INFO << "Configuration loader is ready";
}

void robot_application_t::parse_command_line_arguments() {
    namespace po = boost::program_options;

    arguments_parser_t parser("Allowed options");
    parser.set_arguments(m_arguments);

    parser.add_strategy(m_arg_help_requested);

    parser.add_strategy(m_arg_debug_all);
    parser.add_strategy(m_arg_debug_application);
    parser.add_strategy(m_arg_debug_ball_searcher);
    parser.add_strategy(m_arg_debug_ball_tracker);
    parser.add_strategy(m_arg_debug_game_controller);
    parser.add_strategy(m_arg_debug_motion_manager);
    parser.add_strategy(m_arg_debug_head);
    parser.add_strategy(m_arg_debug_walking);
    parser.add_strategy(m_arg_debug_action);
    parser.add_strategy(m_arg_debug_kicking);
    parser.add_strategy(m_arg_debug_buttons);
    parser.add_strategy(m_arg_debug_leds);
    parser.add_strategy(m_arg_debug_image_source);
    parser.add_strategy(m_arg_debug_camera);
    parser.add_strategy(m_arg_debug_vision_processor);
    parser.add_strategy(m_arg_debug_localization);

    parser.add_strategy(m_arg_config_default);
    parser.add_strategy(m_arg_config_ball_searcher);
    parser.add_strategy(m_arg_config_ball_tracker);
    parser.add_strategy(m_arg_config_game_controller);
    parser.add_strategy(m_arg_config_motion_manager);
    parser.add_strategy(m_arg_config_head);
    parser.add_strategy(m_arg_config_walking);
    parser.add_strategy(m_arg_config_action);
    parser.add_strategy(m_arg_config_kicking);
    parser.add_strategy(m_arg_config_white_ball_vision_processor);
    parser.add_strategy(m_arg_config_localization_field);
    parser.add_strategy(m_arg_config_particle_filter);

    parser.add_strategy(m_arg_white_ball_vision_processor);


    m_arg_help_requested.set_option("help,h", "produce help message");

    m_arg_debug_all.set_option("dbg-all,d", "enable debug output for all components");
    m_arg_debug_application.set_option("dbg-app", "enable debug output for main application");
    m_arg_debug_ball_searcher.set_option("dbg-ball-searcher", "enable debug output for ball searcher");
    m_arg_debug_ball_tracker.set_option("dbg-ball-tracker", "enable debug output for ball tracker");
    m_arg_debug_game_controller.set_option("dbg-game-contoller", "enable debug output for game controller");
    m_arg_debug_motion_manager.set_option("dbg-motion-manager", "enable debug output for motion manager");
    m_arg_debug_head.set_option("dbg-head", "enable debug output for head motion module");
    m_arg_debug_walking.set_option("dbg-walking", "enable debug output for walking motion module");
    m_arg_debug_action.set_option("dbg-action", "enable debug output for action motion module");
    m_arg_debug_kicking.set_option("dbg-kicking", "enable debug output for kicking motion module");
    m_arg_debug_buttons.set_option("dbg-kicking", "enable debug output for buttons");
    m_arg_debug_leds.set_option("dbg-kicking", "enable debug output for LEDs");
    m_arg_debug_camera.set_option("dbg-camera", "enable debug output for camera");
    m_arg_debug_vision_processor.set_option("dbg-cv", "enable debug output for cv");
    m_arg_debug_localization.set_option("dbg-localization", "enable debug output for localization module");

    m_arg_config_default.set_option("cfg,c", "default config file (res/config.ini by default)");
    m_arg_config_ball_tracker.set_option("cfg-ball-tracker", "config file for ball tracker");
    m_arg_config_ball_searcher.set_option("cfg-ball-searcher", "config file for ball searcher");
    m_arg_config_game_controller.set_option("cfg-game-controller", "config file for game controller");
    m_arg_config_motion_manager.set_option("cfg-motion-manager", "config file for motion manager");
    m_arg_config_head.set_option("cfg-head", "config file for head motion module");
    m_arg_config_walking.set_option("cfg-walking", "config file for walking motion module");
    m_arg_config_kicking.set_option("cfg-kicking", "config file for kicking motion module");
    m_arg_config_action.set_option("cfg-action", "path to motion_4096.bin");
    m_arg_config_white_ball_vision_processor.set_option("cfg-cv", "path to cv config");
    m_arg_config_localization_field.set_option("cfg-loc-field", "path to localization field config");
    m_arg_config_particle_filter.set_option("cfg-pf", "path to particle filter config");

#ifdef CROSSCOMPILATION
    m_arg_debug_image_source.set_option("dbg-img-source", "enabled debut output for image source");
    m_arg_config_image_source.set_option("cfg-image-source", "config file for image source");
#endif


    if (!parser.parse() || m_arg_help_requested.is_help_requested()) {
        parser.show_description_to_stream(std::cout);
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
        game_controller_t::get_instance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_motion_manager)
        motion_manager_t::get_instance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_head)
        head_t::get_instance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_walking)
        walking_t::get_instance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_action)
        action_t::get_instance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_kicking)
        kicking_t::get_instance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_buttons)
        buttons_t::get_instance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_leds)
        LEDs_t::GetInstance()->enable_debug(true);
    if (m_arg_debug_all || m_arg_debug_camera)
        camera_t::get_instance()->enable_debug(true);
    // Image source debug placed located in init_cv
    // Localization debug is in init_localization
}

void robot_application_t::read_configuration() {
    if (m_debug) LOG_DEBUG << "Reading configuration...";
    m_configuration_loader.configure_all();
    m_action_configuration_loader.read_motion_file();
    if (m_debug) LOG_INFO << "Reading configuration was finished";
}

void robot_application_t::start_main_loop() {
    if (m_debug) LOG_INFO << "=== Controller was started ===";
    while (is_running()) {
      this->update_image();
    }
    if (m_debug) LOG_INFO << "=== Contoller was finished ===";

}

bool robot_application_t::is_debug() const noexcept {
    return m_debug;
}

void robot_application_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

void robot_application_t::update_image() {
    try {
        camera_t::get_instance()->update_image();
        cv::Mat frame = camera_t::get_instance()->get_image();
        vision_t::get_instance()->set_frame(frame);
        vision_t::get_instance()->process();
    } catch(const image_source_failure& failure) {
        LOG_WARNING << "CV loop iteration failed: " << failure.what();
    }

}

