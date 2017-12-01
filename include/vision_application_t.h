#pragma once


#include <atomic>
#include <memory>
#include <boost/program_options.hpp>
#include <config/command_arguments_t.h>
#include <config/strategies/debug_mode_arguments_parsing_strategy_t.h>
#include <config/strategies/config_path_arguments_parsing_strategy_t.h>
#include <config/strategies/help_arguments_parsing_strategy_t.h>
#include <config/strategies/white_ball_vision_processor_arguments_parsing_strategy_t.h>
#include <config/strategies/white_ball_vision_processor_configuration_strategy_t.h>
#include <config/strategies/particle_filter_configuration_strategy_t.h>
#include "config/configuration_file_loader_t.h"
#include "config/strategies/walking_configuration_strategy_t.h"
#include "config/strategies/motion_manager_configuration_strategy_t.h"

#ifdef CROSSCOMPILE
#include "hw/linux_CM730_t.h"
#include <hw/robot_image_source_t.h>
#include "config/strategies/robot_image_source_configuration_strategy_t.h"
#else
#include "hw/vrep_image_source_t.h"
#include "hw/vrep_connector_t.h"
#endif

namespace drwn {
    enum exec_status {
        EXEC_SUCCESS = 0,
        EXEC_FAILED = 1
    };

    class vision_application_t {
    public:
        static vision_application_t* get_instance();

        void set_program_arguments(const command_arguments_t& arguments);

        int exec();

        void stop();

        bool is_running() const;

        bool is_debug_enabled() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        vision_application_t() = default;

        bool help_message_requested() const noexcept;

        bool try_start();

        void start_main_loop();

        //*** Initialization methods ***//
        void init_cat();

        void initialize();

        void init_cv();

        void init_configuraion_loader();

        void parse_command_line_arguments();

        void apply_debug_arguments();

        void read_configuration();

    private:
        bool m_debug{false}; // Debug output to console

        command_arguments_t m_arguments;

        std::atomic<bool> m_started{ATOMIC_VAR_INIT(false)};
        std::unique_ptr<white_ball_vision_processor_t> m_vision_processor{nullptr};

        //*** Configuration members ***//
        configuration_file_loader_t m_configuration_loader;

        white_ball_vision_processor_configuration_strategy_t m_white_ball_vision_processor_configuration_strategy;

        //*** Command line parsing strategies ***//
        help_arguments_parsing_strategy_t m_arg_help_requested;

        debug_mode_arguments_parsing_strategy_t m_arg_debug_all;
        debug_mode_arguments_parsing_strategy_t m_arg_debug_application;
        debug_mode_arguments_parsing_strategy_t m_arg_debug_camera;
        debug_mode_arguments_parsing_strategy_t m_arg_debug_image_source;
        debug_mode_arguments_parsing_strategy_t m_arg_debug_vision_processor;

        config_path_arguments_parsing_strategy_t m_arg_config_default;
        config_path_arguments_parsing_strategy_t m_arg_config_image_source;
        config_path_arguments_parsing_strategy_t m_arg_config_white_ball_vision_processor;

        white_ball_vision_processor_arguments_parsing_strategy_t m_arg_white_ball_vision_processor;
    };
}


