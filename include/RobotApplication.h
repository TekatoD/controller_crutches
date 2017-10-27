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
        bool m_debug{true}; // Debug output to console
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

        std::string m_config_default;
        std::string m_config_ball_searcher;
        std::string m_config_ball_tracker;
        std::string m_config_game_controller;
        std::string m_config_motion_manager;
        std::string m_config_head;
        std::string m_config_walking;
        std::string m_config_action;
        std::string m_config_kicking;

        ActionConfigurationFileLoader m_action_configuration_loader;

        //*** Platform specific members ***//

#ifdef CROSSCOMPILE
        std::unique_ptr<LinuxCM730> m_linux_cm730{nullptr};
#else
        std::unique_ptr<VrepConnector> m_vrep_connector{nullptr};
#endif
    };
}


