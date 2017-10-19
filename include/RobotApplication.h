#pragma once


#include <atomic>
#include <memory>
#include "LinuxMotionTimer.h"
#include "hw/CM730.h"

#ifdef CROSSCOMPILE
#include "hw/LinuxCM730.h"
#else
#include "hw/VrepConnector.h"
#endif

namespace Robot {
    class RobotApplication {
    public:
        static constexpr char U2D_DEV_NAME0[]{"/dev/ttyUSB0"};
        static constexpr char U2D_DEV_NAME1[]{"/dev/ttyUSB1"};

        static RobotApplication* GetInstance();

        void ParseArguments(int argc, const char** argv);

        void ReadConfiguration();

        int Start();

        void Stop();

        void Initialize();

        bool IsDebug() const noexcept;

        void EnableDebug(bool debug) noexcept;

    private:
        RobotApplication() = default;

        //*** Initialization methods ***//
        void CheckHWStatus();

        void InitCM730();

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

        void CheckFirmware();
    };
}


