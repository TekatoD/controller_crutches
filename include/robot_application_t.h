#pragma once


#include <atomic>
#include <memory>
#include <boost/program_options.hpp>
#include <config/CommandArguments.h>
#include <config/ActionConfigurationFileLoader.h>
#include <config/strategies/BallSearcherConfigurationStrategy.h>
#include <config/strategies/BallTrackerConfigurationStrategy.h>
#include <config/strategies/GameControllerConfigurationStrategy.h>
#include <config/strategies/HeadConfigurationStrategy.h>
#include <config/strategies/DebugModeArgumentsParsingStrategy.h>
#include <config/strategies/ConfigPathArgumentsParsingStrategy.h>
#include <config/strategies/HelpArgumentsParsingStrategy.h>
#include "config/ConfigurationFileLoader.h"
#include "config/strategies/WalkingConfigurationStrategy.h"
#include "config/strategies/MotionManagerConfigurationStrategy.h"
#include "hw/linux_motion_timer_t.h"
#include "hw/CM730_t.h"

#ifdef CROSSCOMPILE
#include "hw/linux_CM730_t.h"
#else
#include "hw/vrep_connector_t.h"
#endif

namespace drwn {
    enum exec_status {
        EXEC_SUCCESS = 0,
        EXEC_FAILED = 1
    };

    class robot_application_t {
    public:
        static constexpr char U2D_DEV_NAME0[]{"/dev/ttyUSB0"};
        static constexpr char DEFAULT_MOTION_FILE[]{"res/motion_4096.bin"};
        static constexpr char DEFAULT_CONFIG_FILE[]{"res/config.ini"};

        static robot_application_t* GetInstance();

        void set_program_arguments(const CommandArguments& arguments);

        int exec();

        void stop();

        bool is_running() const;

        bool is_debug() const noexcept;

        void enable_debug(bool debug) noexcept;

    private:
        robot_application_t() = default;

        bool help_message_requested() const noexcept;

        bool try_start();

        void start_main_loop();

        //*** Initialization methods ***//
        void check_hw_status();

        void initialize();

        void init_CM730();

        void check_firmware();

        void init_motion_manager();

        void init_motion_modules();

        void init_motion_timer();

        void init_game_controller();

        void init_configuraion_loader();

        void parse_command_line_arguments();

        void read_configuration();

    private:
        bool m_debug{false}; // Debug output to console
        bool m_show_help_message{false};

        CommandArguments m_arguments;

        std::atomic<bool> m_started{ATOMIC_VAR_INIT(false)};
        std::unique_ptr<CM730_t> m_cm730{nullptr};
        std::unique_ptr<linux_motion_timer_t> m_motion_timer{nullptr};

        //*** Configuration members ***//
        ConfigurationFileLoader m_configuration_loader;

        BallSearcherConfigurationStrategy m_ball_searcher_configuration_strategy;
        BallTrackerConfigurationStrategy m_ball_tracker_configuration_strategy;
        GameControllerConfigurationStrategy m_game_controller_configuration_strategy;
        HeadConfigurationStrategy m_head_configuration_strategy;
        WalkingConfigurationStrategy m_walking_configuration_strategy;
        MotionManagerConfigurationStrategy m_motion_manager_configuration_strategy;

        //*** Command line parsing strategies ***//
        HelpArgumentsParsingStrategy m_arg_help_requested;

        DebugModeArgumentsParsingStrategy m_arg_debug_all;
        DebugModeArgumentsParsingStrategy m_arg_debug_application;
        DebugModeArgumentsParsingStrategy m_arg_debug_ball_searcher;
        DebugModeArgumentsParsingStrategy m_arg_debug_ball_tracker;
        DebugModeArgumentsParsingStrategy m_arg_debug_game_controller;
        DebugModeArgumentsParsingStrategy m_arg_debug_motion_manager;
        DebugModeArgumentsParsingStrategy m_arg_debug_head;
        DebugModeArgumentsParsingStrategy m_arg_debug_walking;
        DebugModeArgumentsParsingStrategy m_arg_debug_action;
        DebugModeArgumentsParsingStrategy m_arg_debug_kicking;
        DebugModeArgumentsParsingStrategy m_arg_debug_buttons;
        DebugModeArgumentsParsingStrategy m_arg_debug_leds;

        ConfigPathArgumentsParsingStrategy m_arg_config_default;
        ConfigPathArgumentsParsingStrategy m_arg_config_ball_searcher;
        ConfigPathArgumentsParsingStrategy m_arg_config_ball_tracker;
        ConfigPathArgumentsParsingStrategy m_arg_config_game_controller;
        ConfigPathArgumentsParsingStrategy m_arg_config_motion_manager;
        ConfigPathArgumentsParsingStrategy m_arg_config_head;
        ConfigPathArgumentsParsingStrategy m_arg_config_walking;
        ConfigPathArgumentsParsingStrategy m_arg_config_action;
        ConfigPathArgumentsParsingStrategy m_arg_config_kicking;

        ActionConfigurationFileLoader m_action_configuration_loader;

        //*** Platform specific members ***//

#ifdef CROSSCOMPILE
        std::unique_ptr<linux_CM730_t> m_linux_cm730{nullptr};
#else
        std::unique_ptr<vrep_connector_t> m_vrep_connector{nullptr};
#endif

        void apply_debug_arguments();

        void apply_config_arguments();
    };
}


