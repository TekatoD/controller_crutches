#include <iostream>
#include <boost/filesystem.hpp>
#include <config/arguments_parser_t.h>
#include <hw/vrep_image_source_t.h>
#include <opencv/cv.hpp>
#include <vision/vision_t.h>
#include "vision_application_t.h"

using namespace drwn;
namespace fs = boost::filesystem;

vision_application_t* vision_application_t::get_instance() {
    static vision_application_t instance;
    return &instance;
}

void vision_application_t::set_program_arguments(const command_arguments_t& arguments) {
    m_arguments = arguments;
}

int vision_application_t::exec() {
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

bool vision_application_t::help_message_requested() const noexcept {
    return m_arg_help_requested.is_help_requested();
}

bool vision_application_t::try_start() {
    if (help_message_requested()) {
        return false;
    }
    if (is_running()) {
        if (m_debug) LOG_WARNING << "Vision application already started";
        return false;
    }
    m_started.store(true, std::memory_order_release);
    return true;
}

void vision_application_t::stop() {
    m_started.store(false, std::memory_order_release);
}

bool vision_application_t::is_running() const {
    return m_started.load(std::memory_order_acquire);
}

void vision_application_t::initialize() {
    this->apply_debug_arguments();
    if (m_debug) LOG_INFO << "=== Initialization was started ===";
    this->init_cat();
    this->init_cv();
    this->init_configuraion_loader();
    this->read_configuration();
    if (m_debug) LOG_INFO << "=== Initialization was finished ===";
}

void vision_application_t::init_cat() {
    if (m_debug) {
        LOG_DEBUG << "Initializing cat...";
        LOG_DEBUG << "───────────────────────────────────────";
        LOG_DEBUG << "───▐▀▄───────▄▀▌───▄▄▄▄▄▄▄─────────────";
        LOG_DEBUG << "───▌▒▒▀▄▄▄▄▄▀▒▒▐▄▀▀▒██▒██▒▀▀▄──────────";
        LOG_DEBUG << "──▐▒▒▒▒▀▒▀▒▀▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▀▄────────";
        LOG_DEBUG << "──▌▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▄▒▒▒▒▒▒▒▒▒▒▒▒▀▄──────";
        LOG_DEBUG << "▀█▒▒▒█▌▒▒█▒▒▐█▒▒▒▀▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▌─────";
        LOG_DEBUG << "▀▌▒▒▒▒▒▒▀▒▀▒▒▒▒▒▒▀▀▒▒▒▒▒▒▒▒▒▒▒▒▒▒▐───▄▄";
        LOG_DEBUG << "▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▌▄█▒█";
        LOG_DEBUG << "▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▒█▀─";
        LOG_DEBUG << "▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▀───";
        LOG_DEBUG << "▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▌────";
        LOG_DEBUG << "─▌▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▐─────";
        LOG_DEBUG << "─▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▌─────";
        LOG_DEBUG << "──▌▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▐──────";
        LOG_DEBUG << "──▐▄▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▄▌──────";
        LOG_DEBUG << "────▀▄▄▀▀▀▀▀▄▄▀▀▀▀▀▀▀▄▄▀▀▀▀▀▄▄▀────────";
        LOG_INFO << "Cat is ready! ^^";
    }
}

void vision_application_t::init_cv() {
    if (m_debug) LOG_DEBUG << "Initializing CV pipeline...";

    auto vision_processor = std::make_unique<white_ball_vision_processor_t>();

    const std::string& path = m_arg_white_ball_vision_processor.get_dump_images_path();
    if (!path.empty()) {
        if (m_debug) LOG_INFO << "Dumping images to directory: " << path;
        vision_processor->set_dump_directory_path(path);
    }

    const std::string& src_path = m_arg_white_ball_vision_processor.get_source_images_path();
    if (!src_path.empty()) {
        if (m_debug) LOG_INFO << "Vision processor source directory: " << src_path;
        vision_processor->set_source_directory_path(src_path);
    }

    vision_processor->enable_dump_images(m_arg_white_ball_vision_processor.is_dump_images_enabled());
    vision_processor->enable_show_images(m_arg_white_ball_vision_processor.is_display_images_enabled());
    m_vision_processor = std::move(vision_processor);
    vision_t::get_instance()->set_processor(m_vision_processor.get());
    if (m_arg_debug_all || m_arg_debug_vision_processor)
        m_vision_processor->enable_debug(true);

    if (m_debug) LOG_INFO << "CV pipeline is ready";
}

void vision_application_t::init_configuraion_loader() {
    if (m_debug) LOG_DEBUG << "Initializing configuration loader...";
    m_white_ball_vision_processor_configuration_strategy.set_white_ball_vision_processor(m_vision_processor.get());

    m_configuration_loader.set_default_path(m_arg_config_default);
    m_configuration_loader.add_strategy(m_white_ball_vision_processor_configuration_strategy,
                                        m_arg_config_white_ball_vision_processor);

    if (m_debug) LOG_INFO << "Configuration loader is ready";
}

void vision_application_t::parse_command_line_arguments() {
    namespace po = boost::program_options;

    arguments_parser_t parser("Allowed options");
    parser.set_arguments(m_arguments);

    parser.add_strategy(m_arg_help_requested);

    parser.add_strategy(m_arg_debug_all);
    parser.add_strategy(m_arg_debug_application);
    parser.add_strategy(m_arg_debug_vision_processor);

    parser.add_strategy(m_arg_config_default);
    parser.add_strategy(m_arg_config_white_ball_vision_processor);

    parser.add_strategy(m_arg_white_ball_vision_processor);


    m_arg_help_requested.set_option("help,h", "produce help message");

    m_arg_debug_all.set_option("dbg-all,d", "enable debug output for all components");
    m_arg_debug_application.set_option("dbg-app", "enable debug output for main application");
    m_arg_debug_vision_processor.set_option("dbg-cv", "enable debug output for cv");

    m_arg_config_default.set_option("cfg,c", "default config file (res/config.ini by default)");
    m_arg_config_white_ball_vision_processor.set_option("cfg-cv", "path to cv config");

    if (!parser.parse() || m_arg_help_requested.is_help_requested()) {
        parser.show_description_to_stream(std::cout);
    }
}

void vision_application_t::apply_debug_arguments() {
    enable_debug(m_arg_debug_all || m_arg_debug_application);
    m_configuration_loader.enable_debug(m_arg_debug_all);
    // Image source debug placed located in init_cv
}

void vision_application_t::read_configuration() {
    if (m_debug) LOG_DEBUG << "Reading configuration...";
    m_configuration_loader.configure_all();
    if (m_debug) LOG_INFO << "Reading configuration has finished";
}

void vision_application_t::start_main_loop() {
    if (m_debug) LOG_INFO << "=== Main loop has started ===";

    fs::path path(m_vision_processor->get_source_directory_path());
    if (!fs::is_directory(path)) {
        throw std::runtime_error{"Please specify a directory as a source for vision processor"};
    }

    std::vector<fs::path> image_paths;
    std::copy(fs::directory_iterator(path), fs::directory_iterator(), std::back_inserter(image_paths));
    // simple sorting
    std::sort(image_paths.begin(), image_paths.end());

    for (const auto &current_image_path : image_paths) {
        if (!is_running()) { break; }

        if (!fs::is_regular_file(current_image_path)) {
            if (m_debug) {
                LOG_ERROR << "Error reading file: " << current_image_path.string() << " . Skipping...";
            }

            // Skip this image
            continue;
        }

        const std::string image_path_string = current_image_path.string();
        if (m_debug) LOG_DEBUG << "Current image: " << image_path_string;

        cv::Mat frame = cv::imread(image_path_string);
        m_vision_processor->set_frame(frame);
        m_vision_processor->process();

        LOG_INFO << "Waiting for keypress...";
        cv::waitKey(0);
    }

    if (m_debug) LOG_INFO << "=== Main loop has finished ===";
}

bool vision_application_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void vision_application_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}
