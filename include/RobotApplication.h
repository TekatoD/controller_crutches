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
#include "hw/LinuxMotionTimer.h"
#include "hw/CM730.h"

#ifdef CROSSCOMPILE
#include "hw/LinuxCM730.h"
#else
#include "hw/VrepConnector.h"
#endif

namespace Robot {
    enum ExecStatus {
        EXEC_SUCCESS = 0,
        EXEC_FAILED = 1
    };

    class RobotApplication {
    public:
        static constexpr char U2D_DEV_NAME0[]{"/dev/ttyUSB0"};
        static constexpr char DEFAULT_MOTION_FILE[]{"res/motion_4096.bin"};
        static constexpr char DEFAULT_CONFIG_FILE[]{"res/config.ini"};

        static RobotApplication* GetInstance();

        void SetProgramArguments(const CommandArguments& arguments);

        int Exec();

        void Stop();

        bool IsRunning() const;

        bool IsDebug() const noexcept;

        void EnableDebug(bool debug) noexcept;

    private:
        RobotApplication() = default;

        bool HelpMessageRequested() const noexcept;

        bool TryStart();

        void StartMainLoop();

        //*** Initialization methods ***//
        void CheckHWStatus();

        void Initialize();

        void InitCM730();

        void CheckFirmware();

        void InitMotionManager();

        void InitMotionModules();

        void InitMotionTimer();

        void InitGameController();

        void InitConfiguraionLoader();

        void ParseCommandLineArguments();

        void ReadConfiguration();

    private:
        bool m_debug{false}; // Debug output to console
        bool m_show_help_message{false};

        CommandArguments m_arguments;

        std::atomic<bool> m_started{ATOMIC_VAR_INIT(false)};
        std::unique_ptr<CM730> m_cm730{nullptr};
        std::unique_ptr<LinuxMotionTimer> m_motion_timer{nullptr};

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
        std::unique_ptr<LinuxCM730> m_linux_cm730{nullptr};
#else
        std::unique_ptr<VrepConnector> m_vrep_connector{nullptr};
#endif

        void ApplyDebugArguments();

        void ApplyConfigArguments();
    };
}


