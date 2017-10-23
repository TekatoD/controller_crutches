#pragma once


#include <atomic>
#include <memory>
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
        static constexpr char U2D_DEV_NAME1[]{"/dev/ttyUSB1"};
        static constexpr char DEFAULT_MOTION_FILE[]{"res/motion_4096.bin"};
        static constexpr char DEFAULT_CONFIG_FILE[]{"res/config.ini"};

        static RobotApplication* GetInstance();

        void ParseArguments(int argc, const char** argv);

        void ReadConfiguration();

        int Exec();

        void Stop();

        bool IsRunning() const;

        bool IsDebug() const noexcept;

        void EnableDebug(bool debug) noexcept;

    private:

        RobotApplication() = default;

        bool TryStart();

        //*** Initialization methods ***//
        void CheckHWStatus();

        void Initialize();

        void InitCM730();

        void CheckFirmware();

        void InitMotionManager();

        void InitMotionTimer();

        void InitGameController();

    private:
        bool m_debug{true}; // Debug output to console

        std::atomic<bool> m_started{ATOMIC_VAR_INIT(false)};
        std::unique_ptr<CM730> m_cm730{nullptr};
        std::unique_ptr<LinuxMotionTimer> m_motion_timer{nullptr};

        //*** Platform specific members ***//

#ifdef CROSSCOMPILE
        std::unique_ptr<LinuxCM730> m_linux_cm730{nullptr};
#else
        std::unique_ptr<VrepConnector> m_vrep_connector{nullptr};
#endif

        void StartMainLoop();
    };
}


