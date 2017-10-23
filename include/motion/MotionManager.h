#pragma once

#include <list>
#include "MotionStatus.h"
#include "MotionModule.h"
#include "hw/CM730.h"
#include "minIni.h"

#define OFFSET_SECTION "Offset"
#define INVALID_VALUE   -1024.0

namespace Robot {
    class MotionManager {
    public:
        int m_Offset[JointData::NUMBER_OF_JOINTS];

        static MotionManager* GetInstance() {
            static MotionManager instance;
            return &instance;
        }

        bool Initialize(CM730* cm730);

        bool Reinitialize();

        void Process();

        void SetEnable(bool enable);

        bool GetEnable() const { return m_Enabled; }

        void AddModule(MotionModule* module);

        void RemoveModule(MotionModule* module);

        void ResetGyroCalibration() {
            m_CalibrationStatus = 0;
            m_FBGyroCenter = 512;
            m_RLGyroCenter = 512;
        }

        int GetCalibrationStatus() const { return m_CalibrationStatus; }

        void SetJointDisable(int index);

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

        bool IsDebugEnabled() const;

        void EnabledDebug(bool debug);

    private:
        bool m_debug{false};

        std::list<MotionModule*> m_Modules;
        CM730* m_CM730{nullptr};
        bool m_ProcessEnable{false};
        bool m_Enabled{false};
        int m_FBGyroCenter{512};
        int m_RLGyroCenter{512};
        int m_CalibrationStatus{0};

        bool m_IsRunning{false};
        bool m_IsThreadRunning{false};

        MotionManager();
    };
}
